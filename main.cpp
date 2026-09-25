// compat_layer - Esqueleto del "programa central".
// Carga un ELF de PSP (ya descifrado) en memoria de la PS2, aplica sus
// relocalizaciones y salta a su punto de entrada. TODAVIA NO TRADUCE
// NADA - esto es solo el mecanismo de carga + reubicacion. El salto al
// final va a crashear/colgar la consola, y esta bien que asi sea: ese
// es el punto exacto donde mas adelante va a vivir el traductor de
// instrucciones.

#include <tamtypes.h>
#include <kernel.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sifrpc.h>
#include <dma.h>
#include <dma_tags.h>
#include <gif_tags.h>
#include <gs_psm.h>
#include <packet.h>
#include <draw.h>
#include <graph.h>
#include "traductor.h"


// ============================================================
// Formato de cabecera ELF de 32 bits
// ============================================================

typedef struct {
    unsigned char e_ident[16];
    u16 e_type;
    u16 e_machine;
    u32 e_version;
    u32 e_entry;
    u32 e_phoff;
    u32 e_shoff;
    u32 e_flags;
    u16 e_ehsize;
    u16 e_phentsize;
    u16 e_phnum;
    u16 e_shentsize;
    u16 e_shnum;
    u16 e_shstrndx;
} Elf32_Ehdr;

typedef struct {
    u32 p_type;
    u32 p_offset;
    u32 p_vaddr;
    u32 p_paddr;
    u32 p_filesz;
    u32 p_memsz;
    u32 p_flags;
    u32 p_align;
} Elf32_Phdr;

typedef struct {
    u32 sh_name;
    u32 sh_type;
    u32 sh_flags;
    u32 sh_addr;
    u32 sh_offset;
    u32 sh_size;
    u32 sh_link;
    u32 sh_info;
    u32 sh_addralign;
    u32 sh_entsize;
} Elf32_Shdr;

typedef struct {
    u32 r_offset;
    u32 r_info;
} Elf32_Rel;

#define PT_LOAD 1

// --- Tipos de reloc MIPS que nos importan ---
#define R_MIPS_32    2
#define R_MIPS_26    4
#define R_MIPS_HI16  5
#define R_MIPS_LO16  6

// r_info en el formato de PRX de PSP se parte asi:
//   bits 0-7   : tipo de reloc (R_MIPS_*)
//   bits 8-15  : OFS_BASE -> a que segmento pertenece r_offset
//   bits 16-23 : ADDR_BASE -> a que segmento apunta el valor calculado
// (ADDR_BASE no lo necesitamos para aplicar el parche: como todo el
//  modulo vive en un unico bloque contiguo, 'delta' es el mismo sin
//  importar el segmento. Solo OFS_BASE importa, para saber DONDE esta
//  el dato/instruccion a corregir.)
#define ELF32_R_TYPE(info)     ((info) & 0xFF)
#define ELF32_R_OFS_BASE(info) (((info) >> 8) & 0xFF)

#define MAX_SEGMENTOS 16

// ANTES de reservar bloqueUnico, en main.cpp:
// El modulo ocupa 5.4MB. Lo ponemos en una zona baja conocida.
// PS2 tiene 32MB. El IOP usa 2MB. EE kernel usa algo al inicio.
// PS2SDK pone el heap del programa alrededor de 0x006xxxxx.
// Nosotros queremos el modulo en 0x00200000 (2MB) para que
// _end caiga alrededor de 0x00760000, bien dentro de la RAM.
#define DIRECCION_MODULO_FIJA 0x00200000

static u8 bloqueModuloPSP[8000000] __attribute__((aligned(64)));

// ============================================================
// Emparejamiento HI16/LO16
// ============================================================
// No podemos asumir que cada HI16 tiene su LO16 inmediatamente despues
// en la lista: en la practica, varios LO16 pueden reutilizar el MISMO
// HI16 anterior (visto en los logs: entradas con el mismo symidx que
// comparten un HI16 previo sin uno nuevo en el medio). La regla real:
// cada LO16 se empareja con el HI16 mas reciente que tenga el MISMO
// symidx completo (OFS_BASE+ADDR_BASE), sin importar cuantos LO16 ya
// lo hayan usado antes, y sin importar si en el medio aparecieron
// HI16/LO16 de OTRO symidx.
//
// Por eso usamos un mapa "ultimo HI16 visto, indexado por symidx" en
// vez de una pila que se vacia al usarse. Y guardamos el valor
// ORIGINAL (sin parchear) de esa instruccion HI16, porque si un mismo
// HI16 se reutiliza para varios LO16, cada calculo tiene que partir
// siempre del mismo dato original - si partieramos del dato ya
// parcheado por un uso anterior, el delta se acumularia mal.

#define MAX_SYMIDX 4096

typedef struct {
    int valido;
    u32 direccionRealDelHi;   // donde esta en memoria la instruccion HI16
    u32 immOriginal;          // los 16 bits altos ORIGINALES (antes de parchear)
    int yaTuvoUnPatch;        // para detectar y avisar si un reuso pide un HI16 distinto
    u32 ultimoNuevoHiEscrito;
} HiPendiente;

static HiPendiente tablaHi[MAX_SYMIDX];

// ============================================================
// Aplicar todas las relocs de UNA seccion .rel.*
// ============================================================
static void aplicarRelocsDeSeccion(FILE *f, u32 offsetEnArchivo, u32 tamanoSeccion,
                                    void *bloqueUnico, u32 vaddrMinimo, s32 delta,
                                    u32 *vaddrSegmentos, int cantSegmentos,
                                    u32 *contadorAplicadas, u32 *contadorHuerfanas,
                                    u32 *contadorTipoDesconocido) {
    u32 cantidadEntradas = tamanoSeccion / sizeof(Elf32_Rel);

    for (u32 i = 0; i < cantidadEntradas; i++) {
        Elf32_Rel rel;
        fseek(f, offsetEnArchivo + i * sizeof(Elf32_Rel), SEEK_SET);
        fread(&rel, sizeof(Elf32_Rel), 1, f);

        u32 tipo    = ELF32_R_TYPE(rel.r_info);
        u32 ofsBase = ELF32_R_OFS_BASE(rel.r_info);
        u32 symidx  = rel.r_info >> 8; // clave completa para el mapa HI16/LO16

        if (ofsBase >= (u32)cantSegmentos) {
            printf("  RELOC: OFS_BASE invalido (%u) en entrada %u - salteada\n",
                   (unsigned int)ofsBase, (unsigned int)i);
            continue;
        }

        u32 vaddrObjetivo = vaddrSegmentos[ofsBase] + rel.r_offset;
        u32 *destino = (u32 *)((u8 *)bloqueUnico + (vaddrObjetivo - vaddrMinimo));

        switch (tipo) {
            case R_MIPS_32: {
                *destino += (u32)delta;
                (*contadorAplicadas)++;
                break;
            }

            case R_MIPS_26: {
                u32 instr = *destino;
                u32 indiceOriginal = instr & 0x03FFFFFF;
                u32 nuevoIndice = (indiceOriginal + ((u32)delta >> 2)) & 0x03FFFFFF;
                *destino = (instr & 0xFC000000) | nuevoIndice;
                (*contadorAplicadas)++;
                break;
            }

            case R_MIPS_HI16: {
                if (symidx >= MAX_SYMIDX) {
                    printf("  RELOC: symidx fuera de rango en HI16 (%u)\n", (unsigned int)symidx);
                    break;
                }
                u32 instr = *destino;
                tablaHi[symidx].valido = 1;
                tablaHi[symidx].direccionRealDelHi = (u32)destino;
                tablaHi[symidx].immOriginal = instr & 0xFFFF;
                tablaHi[symidx].yaTuvoUnPatch = 0;
                (*contadorAplicadas)++;
                break;
            }

            case R_MIPS_LO16: {
                if (symidx >= MAX_SYMIDX || !tablaHi[symidx].valido) {
                    (*contadorHuerfanas)++;
                    break;
                }

                u32 instrLo = *destino;
                s16 valLoOriginal = (s16)(instrLo & 0xFFFF); // con signo, importante

                u32 hiImmOriginal = tablaHi[symidx].immOriginal;
                u32 valorOriginal = (hiImmOriginal << 16) + (s32)valLoOriginal;
                u32 valorNuevo = valorOriginal + (u32)delta;

                u32 nuevoHi = (valorNuevo + 0x8000) >> 16;
                u32 nuevoLo = valorNuevo & 0xFFFF;

                // Parchear la instruccion LO16 (la de esta entrada)
                *destino = (instrLo & 0xFFFF0000) | nuevoLo;

                // Parchear (o re-parchear) la instruccion HI16 asociada
                u32 *destinoHi = (u32 *)tablaHi[symidx].direccionRealDelHi;

                if (tablaHi[symidx].yaTuvoUnPatch &&
                    tablaHi[symidx].ultimoNuevoHiEscrito != nuevoHi) {
                    // Caso raro: dos LO16 que comparten un mismo HI16 piden
                    // valores de HI16 distintos (por un acarreo distinto).
                    // No deberia pasar casi nunca con este toolchain, pero
                    // lo dejamos loggeado para poder investigarlo puntualmente
                    // si aparece, en vez de fallar en silencio.
                    printf("  RELOC: HI16 compartido con valores distintos en 0x%08X "
                           "(anterior=0x%04X nuevo=0x%04X) symidx=%u\n",
                           (unsigned int)destinoHi,
                           (unsigned int)tablaHi[symidx].ultimoNuevoHiEscrito,
                           (unsigned int)nuevoHi, (unsigned int)symidx);
                }

                *destinoHi = (*destinoHi & 0xFFFF0000) | (nuevoHi & 0xFFFF);
                tablaHi[symidx].yaTuvoUnPatch = 1;
                tablaHi[symidx].ultimoNuevoHiEscrito = nuevoHi;

                (*contadorAplicadas)++;
                break;
            }

            default:
                (*contadorTipoDesconocido)++;
                break;
        }
    }
}

// ============================================================
// main
// ============================================================

int main(int argc, char *argv[]) {
    SifInitRpc(0);

    // _IONBF (sin buffer) generaba demasiado trafico RPC hacia el IOP en
    // cada printf (tty: no es un archivo normal aca). _IOLBF alcanza para
    // ver el log al instante (flush en cada '\n') sin ese estres extra.
    setvbuf(stdout, NULL, _IOLBF, 4096);

    printf("=== compat_layer - loader esqueleto ===\n");

    // ============================================================
    // PRUEBA DE VIDEO - paso 1 de GE: pintar la pantalla con un color
    // solido de VERDAD (via GS + DMA), sin tocar listas de dibujo
    // todavia. El intento anterior con graph_set_bgcolor solo pintaba
    // el borde/overscan, no el area real de imagen - por eso no se veia
    // nada aunque PCSX2 reconociera bien la configuracion de CRTC. Este
    // usa el patron oficial de ps2sdk (draw_setup_environment + draw_clear
    // + DMA), el mismo mecanismo base que despues va a usar el render
    // real de GE.
    // ============================================================
    {
        framebuffer_t frame;
        zbuffer_t zbuf;

        frame.width = 640;
        frame.height = 448; // NTSC entrelazado tipico
        frame.mask = 0;
        frame.psm = GS_PSM_32;
        frame.address = 0; // unico framebuffer por ahora, arranca en VRAM 0

        zbuf.enable = 0;
        zbuf.method = 0;
        zbuf.address = 0;
        zbuf.mask = 1;
        zbuf.zsm = 0;

        graph_initialize(frame.address, frame.width, frame.height, frame.psm, 0, 0);

        dma_channel_initialize(DMA_CHANNEL_GIF, NULL, 0);

        packet_t *packet = packet_init(50, PACKET_NORMAL);
        qword_t *q = packet->data;
        q = draw_setup_environment(q, 0, &frame, &zbuf);
        q = draw_clear(q, 0, 0.0f, 0.0f, (float)frame.width, (float)frame.height, 0, 200, 60); // verde
        dma_channel_send_normal(DMA_CHANNEL_GIF, packet->data, q - packet->data, 0, 0);
        dma_wait_fast(); // esperar a que termine antes de liberar el packet
        packet_free(packet);

        graph_enable_output();

        printf("TRADUCTOR: video inicializado (%dx%d, PSM=%d) - clear real enviado por DMA, deberia verse VERDE\n",
            frame.width, frame.height, frame.psm);
    }

    const char *prefijos[] = { "mass:", "host:" };
    FILE *f = NULL;
    char rutaUsada[64];

    for (int i = 0; i < 2 && f == NULL; i++) {
        snprintf(rutaUsada, sizeof(rutaUsada), "%sULUS10509_EBOOT.BIN", prefijos[i]);
        f = fopen(rutaUsada, "rb");
    }

    if (f == NULL) {
        printf("No se pudo abrir el ELF de PSP (probado mass: y host:).\n");
        while (1) {}
    }

    printf("Abierto: %s\n", rutaUsada);

    Elf32_Ehdr header;
    fread(&header, sizeof(Elf32_Ehdr), 1, f);

    if (header.e_ident[0] != 0x7F || header.e_ident[1] != 'E' ||
        header.e_ident[2] != 'L'  || header.e_ident[3] != 'F') {
        printf("ERROR: no es un ELF valido.\n");
        fclose(f);
        while (1) {}
    }

    printf("Punto de entrada (e_entry): 0x%08X\n", (unsigned int)header.e_entry);
    printf("Cantidad de program headers: %d\n", header.e_phnum);

    if (header.e_phnum == 0) {
        printf("ERROR: el ELF no tiene program headers.\n");
        fclose(f);
        while (1) {}
    }

    // --- PRIMERA PASADA: medir rango total y guardar el vaddr de cada PT_LOAD ---

    u32 vaddrMinimo = 0xFFFFFFFF;
    u32 vaddrMaximo = 0;
    u32 vaddrSegmentos[MAX_SEGMENTOS];
    u32 offsetSegmentos[MAX_SEGMENTOS];
    u32 fileszSegmentos[MAX_SEGMENTOS];
    int cantSegmentos = 0;

    fseek(f, header.e_phoff, SEEK_SET);
    for (int i = 0; i < header.e_phnum; i++) {
        Elf32_Phdr ph;
        fread(&ph, sizeof(Elf32_Phdr), 1, f);
        if (ph.p_type != PT_LOAD) continue;

        if (cantSegmentos < MAX_SEGMENTOS) {
            vaddrSegmentos[cantSegmentos] = ph.p_vaddr;
            offsetSegmentos[cantSegmentos] = ph.p_offset;
            fileszSegmentos[cantSegmentos] = ph.p_filesz;
            cantSegmentos++;
        }

        if (ph.p_vaddr < vaddrMinimo) vaddrMinimo = ph.p_vaddr;
        u32 finDeSegmento = ph.p_vaddr + ph.p_memsz;
        if (finDeSegmento > vaddrMaximo) vaddrMaximo = finDeSegmento;
    }

    if (cantSegmentos == 0) {
        printf("ERROR: no hay segmentos PT_LOAD.\n");
        fclose(f);
        while (1) {}
    }

    //nuevo lol
    printf("ELF: %d segmentos PT_LOAD\n", cantSegmentos);

    for (int i = 0; i < cantSegmentos; i++) {
        printf(
            "  LOAD[%d]: vaddr=0x%08X offset=0x%08X filesz=0x%08X\n",
            i,
            (unsigned int)vaddrSegmentos[i],
            (unsigned int)offsetSegmentos[i],
            (unsigned int)fileszSegmentos[i]
        );
    }
    //no nuevo ->

    u32 tamanoTotal = vaddrMaximo - vaddrMinimo;
    printf("Rango virtual del modulo: 0x%08X - 0x%08X (tamano: %u bytes)\n",
           (unsigned int)vaddrMinimo, (unsigned int)vaddrMaximo, (unsigned int)tamanoTotal);

    #define PSP_PARTITION_POOL_SIZE (2 * 1024 * 1024) // 2MB
    void *pspPartitionPool = malloc(PSP_PARTITION_POOL_SIZE);
    if (!pspPartitionPool) {
        printf("ERROR: no hay memoria para partition pool\n");
        while(1) {}
    }
    memset(pspPartitionPool, 0, PSP_PARTITION_POOL_SIZE);
    traductor_setPartitionPool((u8*)pspPartitionPool, PSP_PARTITION_POOL_SIZE);

    // Reservamos EXTRA (0x10000 de mas) para poder elegir, dentro de ese
    // bloque, un punto de partida tal que el DELTA resultante sea multiplo
    // de 0x10000 (64KB). Esto es clave para las relocs HI16/LO16: si el
    // delta desplaza todo en bloques completos de 64KB, los 16 bits bajos
    // de CUALQUIER direccion no cambian - por lo tanto ningun grupo de
    // instrucciones que compartia un mismo HI16 antes de relocalizar puede
    // dejar de compartirlo despues. Sin este alineado, un delta "cualquiera"
    // puede hacer que direcciones que antes redondeaban igual empiecen a
    // redondear distinto, generando conflictos reales de HI16 compartido.
    void *bufferBruto = bloqueModuloPSP;
    u32 tamanoDisponible = sizeof(bloqueModuloPSP);
    if (tamanoTotal + 0x10000 > tamanoDisponible) {
        printf("ERROR: modulo demasiado grande para el buffer estatico\n");
        while(1) {}
    }

    // Elegimos, dentro de bufferBruto, la primera direccion tal que sus
    // 16 bits bajos coincidan con los 16 bits bajos de vaddrMinimo (eso
    // es justamente lo que hace que delta sea multiplo de 0x10000).
    u32 base = (u32)bufferBruto;
    u32 objetivoBits16Bajos = vaddrMinimo & 0xFFFF;
    u32 candidato = (base & 0xFFFF0000) | objetivoBits16Bajos;
    if (candidato < base) {
        candidato += 0x10000; // si quedo antes del buffer, subimos una pagina de 64KB
    }
    void *bloqueUnico = (void *)candidato;
    memset(bloqueUnico, 0, tamanoTotal);

    s32 delta = (s32)((u32)bloqueUnico - vaddrMinimo);

    u32 heapOffset = (tamanoTotal + 63) & ~63;
    u32 heapSize   = (u32)sizeof(bloqueModuloPSP) - heapOffset;
    printf("PSP heap pool: addr=0x%08X size=0x%08X\n",
        (unsigned int)((u8*)bloqueUnico + heapOffset),
        (unsigned int)heapSize);
    traductor_setPartitionPool((u8*)bloqueUnico + heapOffset, heapSize);

    printf("Bloque unico reservado en: %p\n", bloqueUnico);
    printf("Delta (real - virtual): 0x%08X\n", (unsigned int)delta);

    // --- SEGUNDA PASADA: copiar cada segmento a su lugar dentro del bloque unico ---

    fseek(f, header.e_phoff, SEEK_SET);
    for (int i = 0; i < header.e_phnum; i++) {
        Elf32_Phdr ph;
        fread(&ph, sizeof(Elf32_Phdr), 1, f);
        if (ph.p_type != PT_LOAD) continue;

        printf("Segmento: vaddr=0x%08X filesz=%u memsz=%u\n",
               (unsigned int)ph.p_vaddr, (unsigned int)ph.p_filesz, (unsigned int)ph.p_memsz);

        void *destino = (char *)bloqueUnico + (ph.p_vaddr - vaddrMinimo);

        long posActual = ftell(f);
        fseek(f, ph.p_offset, SEEK_SET);
        fread(destino, 1, ph.p_filesz, f);
        fseek(f, posActual, SEEK_SET);
    }

    // --- Leer la tabla de secciones y encontrar todas las .rel.* ---

    if (header.e_shnum == 0) {
        printf("ADVERTENCIA: el ELF no tiene tabla de secciones - no se pueden aplicar relocs.\n");
    } else {
        Elf32_Shdr *secciones = (Elf32_Shdr *)malloc(sizeof(Elf32_Shdr) * header.e_shnum);
        fseek(f, header.e_shoff, SEEK_SET);
        fread(secciones, sizeof(Elf32_Shdr), header.e_shnum, f);

        Elf32_Shdr shstrtabHdr = secciones[header.e_shstrndx];
        char *shstrtab = (char *)malloc(shstrtabHdr.sh_size);
        fseek(f, shstrtabHdr.sh_offset, SEEK_SET);
        fread(shstrtab, 1, shstrtabHdr.sh_size, f);

        u32 totalAplicadas = 0;
        u32 totalHuerfanas = 0;
        u32 totalDesconocidas = 0;

        u32 *p = (u32 *)((u8 *)bloqueUnico + (0x00000140 - vaddrMinimo));

        printf(
            "DEBUG JAL: antes relocs vaddr=0x00000140 instr=0x%08X\n",
            (unsigned int)*p
        );

        u32 libStubAddr = 0, libStubSize = 0;

        for (int i = 0; i < header.e_shnum; i++) {
            const char *nombre = shstrtab + secciones[i].sh_name;

            if (strcmp(nombre, ".lib.stub") == 0) {
                libStubAddr = secciones[i].sh_addr;
                libStubSize = secciones[i].sh_size;
            }

            if (strncmp(nombre, ".rel", 4) != 0) continue;
            if (secciones[i].sh_size == 0) continue;

            printf("Aplicando relocs de %s (%u bytes)...\n",
                   nombre, (unsigned int)secciones[i].sh_size);

            aplicarRelocsDeSeccion(f, secciones[i].sh_offset, secciones[i].sh_size,
                                    bloqueUnico, vaddrMinimo, delta,
                                    vaddrSegmentos, cantSegmentos,
                                    &totalAplicadas, &totalHuerfanas, &totalDesconocidas);
        }

        printf("Relocs aplicadas: %u | huerfanas (LO16 sin HI16): %u | tipo desconocido: %u\n",
               (unsigned int)totalAplicadas, (unsigned int)totalHuerfanas, (unsigned int)totalDesconocidas);

        u32 indiceGlobalImport = 0;

        if (libStubAddr != 0 && libStubSize != 0) {
            u8 *baseLibStub = (u8 *)bloqueUnico + (libStubAddr - vaddrMinimo);
            u32 offsetEnStub = 0;

            while (offsetEnStub < libStubSize) {
                u32 nombreLib, tablaNids, primerStub;
                u16 numFunc;
                u8 size;

                memcpy(&nombreLib, baseLibStub + offsetEnStub + 0, 4);
                memcpy(&size,      baseLibStub + offsetEnStub + 8, 1);
                memcpy(&numFunc,   baseLibStub + offsetEnStub + 10, 2);
                memcpy(&tablaNids, baseLibStub + offsetEnStub + 12, 4);
                memcpy(&primerStub,baseLibStub + offsetEnStub + 16, 4);

                // nombreLib/tablaNids/primerStub YA son direcciones REALES
                // (.rel.lib.stub ya les aplico el delta arriba).
                const char *nombreLibStr = (const char *)nombreLib;

                for (u16 fi = 0; fi < numFunc; fi++) {
                    u32 nid;
                    memcpy(&nid, (void *)(tablaNids + fi * 4), 4);

                    u32 direccionDelaySlot = primerStub + fi * 8 + 4;

                    if (indiceGlobalImport < TRADUCTOR_MAX_IMPORTS) {
                        traductor_registrarImport(indiceGlobalImport, nid, nombreLibStr);

                        u32 syscallInstr = (indiceGlobalImport << 6) | 0x0C;
                        memcpy((void *)direccionDelaySlot, &syscallInstr, 4);

                        indiceGlobalImport++;
                    } else {
                        printf("AVISO: TRADUCTOR_MAX_IMPORTS alcanzado, descartado nid=0x%08X\n",
                            (unsigned int)nid);
                    }
                }

                if (size == 0) {
                    printf("ERROR: size=0 en .lib.stub, corto el parseo\n");
                    break;
                }
                offsetEnStub += size * 4;
            }

            printf("Imports registrados: %u\n", (unsigned int)indiceGlobalImport);
        }

        // ============================================================
        // CARGAR kjfs.prx COMO MODULO ADICIONAL (igual que el EBOOT)
        // ============================================================
        // Se carga en su propio bloque, se relocaliza con su delta, se
        // registran sus IMPORTS (para que sus llamadas al sistema caigan en
        // el interprete) y sus EXPORTS (para que el EBOOT pueda llamar las
        // funciones de kjfs). Al registrar los exports, el interprete
        // "secuestra" el salto y ejecuta el codigo real de kjfs cuando el
        // EBOOT lo invoca - eso es lo que destraba el loop infinito.
        {
            static u8 bloqueKjfs[512 * 1024] __attribute__((aligned(65536)));
            const char *prefijosK[] = { "mass:", "host:" };
            FILE *fk = NULL;
            char rutaK[64];
            for (int i = 0; i < 2 && fk == NULL; i++) {
                snprintf(rutaK, sizeof(rutaK), "%skjfs.prx", prefijosK[i]);
                fk = fopen(rutaK, "rb");
            }
            if (fk == NULL) {
                printf("AVISO: no se pudo abrir kjfs.prx - el modulo kjfs NO se cargara\n");
            } else {
                Elf32_Ehdr hk;
                fread(&hk, sizeof(Elf32_Ehdr), 1, fk);
                if (hk.e_ident[0] != 0x7F || hk.e_ident[1] != 'E' ||
                    hk.e_ident[2] != 'L'  || hk.e_ident[3] != 'F') {
                    printf("AVISO: kjfs.prx no es un ELF valido\n");
                    fclose(fk); fk = NULL;
                }
                if (fk) {
                    u32 vMinK = 0xFFFFFFFF, vMaxK = 0;
                    u32 vsK[MAX_SEGMENTOS];
                    int csK = 0;
                    fseek(fk, hk.e_phoff, SEEK_SET);
                    for (int i = 0; i < hk.e_phnum; i++) {
                        Elf32_Phdr ph;
                        fread(&ph, sizeof(Elf32_Phdr), 1, fk);
                        if (ph.p_type != PT_LOAD) continue;
                        if (csK < MAX_SEGMENTOS) {
                            vsK[csK] = ph.p_vaddr;
                            csK++;
                        }
                        if (ph.p_vaddr < vMinK) vMinK = ph.p_vaddr;
                        u32 fin = ph.p_vaddr + ph.p_memsz;
                        if (fin > vMaxK) vMaxK = fin;
                    }
                    u32 tamK = vMaxK - vMinK;

                    // Alinear el bloque a 0x10000 para que delta sea multiplo de
                    // 0x10000 (igual razon que el EBOOT: HI16/LO16 estables).
                    u32 baseK = ((u32)bloqueKjfs + 0xFFFF) & 0xFFFF0000;
                    void *bloqueK = (void *)baseK;
                    s32 deltaK = (s32)(baseK - vMinK);
                    memset(bloqueK, 0, tamK);

                    fseek(fk, hk.e_phoff, SEEK_SET);
                    for (int i = 0; i < hk.e_phnum; i++) {
                        Elf32_Phdr ph;
                        fread(&ph, sizeof(Elf32_Phdr), 1, fk);
                        if (ph.p_type != PT_LOAD) continue;
                        void *dst = (char *)bloqueK + (ph.p_vaddr - vMinK);
                        long pos = ftell(fk);
                        fseek(fk, ph.p_offset, SEEK_SET);
                        fread(dst, 1, ph.p_filesz, fk);
                        fseek(fk, pos, SEEK_SET);
                    }

                    Elf32_Shdr *secsK = NULL;
                    char *shstrK = NULL;
                    u32 libEntK = 0, libEntEndK = 0, libStubK = 0, libStubEndK = 0, gpK = 0, modStartK = 0;
                    if (hk.e_shnum > 0) {
                        secsK = (Elf32_Shdr *)malloc(sizeof(Elf32_Shdr) * hk.e_shnum);
                        fseek(fk, hk.e_shoff, SEEK_SET);
                        fread(secsK, sizeof(Elf32_Shdr), hk.e_shnum, fk);
                        Elf32_Shdr shstrHdrK = secsK[hk.e_shstrndx];
                        shstrK = (char *)malloc(shstrHdrK.sh_size);
                        fseek(fk, shstrHdrK.sh_offset, SEEK_SET);
                        fread(shstrK, 1, shstrHdrK.sh_size, fk);
                    }

                    // Aplicar relocs de kjfs
                    u32 caK = 0, chK = 0, ctK = 0;
                    if (secsK) {
                        for (int i = 0; i < hk.e_shnum; i++) {
                            const char *nm = shstrK + secsK[i].sh_name;
                            if (strncmp(nm, ".rel", 4) == 0 && secsK[i].sh_size) {
                                aplicarRelocsDeSeccion(fk, secsK[i].sh_offset, secsK[i].sh_size,
                                    bloqueK, vMinK, deltaK, vsK, csK,
                                    &caK, &chK, &ctK);
                            }
                            if (strcmp(nm, ".lib.stub") == 0) {
                                libStubK = secsK[i].sh_addr;
                                libStubEndK = secsK[i].sh_addr + secsK[i].sh_size;
                            }
                            if (strcmp(nm, ".lib.ent") == 0) {
                                libEntK = secsK[i].sh_addr;
                                libEntEndK = secsK[i].sh_addr + secsK[i].sh_size;
                            }
                            if (strcmp(nm, ".rodata.sceModuleInfo") == 0) {
                                u8 *mi = (u8 *)bloqueK + (secsK[i].sh_addr - vMinK);
                                memcpy(&gpK, mi + 0x20, 4);        // gp_value en offset 0x20
                                memcpy(&modStartK, mi + 0x34, 4);  // module_start en offset 0x34
                            }
                        }
                    }
                    printf("KJFS: relocs aplicadas=%u | gp(vaddr)=0x%08X\n", (unsigned int)caK, (unsigned int)gpK);

                    u32 realMinK = (u32)bloqueK;
                    u32 realMaxK = (u32)bloqueK + tamK;
                    u32 gpRealK = gpK + deltaK;
                    // kjfs no tiene module_start (es 0), no sumar deltaK para no ejecutar basura
                    u32 modStartRealK = (modStartK != 0) ? (modStartK + deltaK) : 0;

                    int moduloIdx = traductor_registrarModulo(realMinK, realMaxK, vMinK, vMaxK, gpRealK, modStartRealK, 1);

                    // Registrar EXPORTS de kjfs (los que el EBOOT llama)
                    u32 regExp = 0;
                    if (moduloIdx >= 0 && libEntK != 0) {
                        u8 *baseLE = (u8 *)bloqueK + (libEntK - vMinK);
                        u32 pos = 0;
                        u32 fin = libEntEndK - libEntK;
                        while (pos + 16 <= fin) {
                            u32 name_ptr, exports_ptr;
                            u16 version, attribute;
                            u8 ent_size, var_count, func_count, res;
                            memcpy(&name_ptr,    baseLE + pos + 0,  4);
                            memcpy(&version,     baseLE + pos + 4,  2);
                            memcpy(&attribute,   baseLE + pos + 6,  2);
                            memcpy(&ent_size,    baseLE + pos + 8,  1);
                            memcpy(&var_count,   baseLE + pos + 9,  1);
                            memcpy(&func_count,  baseLE + pos + 10, 1);
                            memcpy(&res,         baseLE + pos + 11, 1);
                            memcpy(&exports_ptr, baseLE + pos + 12, 4);
                            u32 total = var_count + func_count;
                            if (total > 0 && exports_ptr) {
                                u32 *nids = (u32 *)exports_ptr;
                                u32 *addrs = (u32 *)(exports_ptr + total * 4);
                                for (u32 fi = 0; fi < total; fi++) {
                                    // addrs[fi] YA es direccion REAL (el .rel.lib.ent
                                    // ya le aplico el delta, igual que los punteros
                                    // del .lib.stub del EBOOT). No sumar delta de nuevo.
                                    u32 ar = addrs[fi];
                                    if (ar >= realMinK && ar < realMaxK) {
                                        traductor_registrarExport(nids[fi], ar, moduloIdx);
                                        regExp++;
                                    }
                                }
                            }
                            if (ent_size == 0) break;
                            pos += ent_size * 4;
                        }
                    }
                    printf("KJFS: exports registrados=%u\n", (unsigned int)regExp);

                    // Registrar IMPORTS de kjfs (continuando el indice global)
                    if (libStubK != 0) {
                        u8 *baseLS = (u8 *)bloqueK + (libStubK - vMinK);
                        u32 pos = 0;
                        u32 fin = libStubEndK - libStubK;
                        while (pos + 16 <= fin) {
                            u32 nombreLib, tablaNids, primerStub;
                            u16 numFunc; u8 size;
                            memcpy(&nombreLib,  baseLS + pos + 0,  4);
                            memcpy(&size,       baseLS + pos + 8,  1);
                            memcpy(&numFunc,    baseLS + pos + 10, 2);
                            memcpy(&tablaNids,  baseLS + pos + 12, 4);
                            memcpy(&primerStub, baseLS + pos + 16, 4);
                            const char *libNameStr = (const char *)nombreLib;
                            for (u16 fi = 0; fi < numFunc; fi++) {
                                u32 nid;
                                memcpy(&nid, (void *)(tablaNids + fi * 4), 4);
                                u32 dslot = primerStub + fi * 8 + 4;
                                if (indiceGlobalImport < TRADUCTOR_MAX_IMPORTS) {
                                    traductor_registrarImport(indiceGlobalImport, nid, libNameStr);
                                    u32 syscallInstr = (indiceGlobalImport << 6) | 0x0C;
                                    memcpy((void *)dslot, &syscallInstr, 4);
                                    indiceGlobalImport++;
                                } else {
                                    printf("AVISO: MAX_IMPORTS alcanzado (kjfs) nid=0x%08X\n", (unsigned int)nid);
                                }
                            }
                            if (size == 0) break;
                            pos += size * 4;
                        }
                    }
                    printf("KJFS: imports registrados, indice global ahora=%u\n", (unsigned int)indiceGlobalImport);

                    // NOTA: el module_start de kjfs se ejecuta DENTRO de
                    // traductor_ejecutar (ver ahi), para que los hilos que
                    // kjfs pueda crear al inicializar no se pierdan cuando se
                    // arma el hilo principal.

                    if (secsK)  free(secsK);
                    if (shstrK) free(shstrK);
                    fclose(fk);
                }
            }
        }

        free(shstrtab);
        free(secciones);
    }

    u32 *p2 = (u32 *)((u8 *)bloqueUnico + (0x00000140 - vaddrMinimo));
    printf("DEBUG JAL: despues relocs vaddr=0x00000140 instr=0x%08X\n", (unsigned int)*p2);

    u32 jal = *p2;

    u32 targetField = jal & 0x03FFFFFF;
    u32 jalDestino = ((0x140 + 4) & 0xF0000000) | (targetField << 2);

    printf(
        "DEBUG JAL: targetField=0x%08X destino=0x%08X\n",
        (unsigned int)targetField,
        (unsigned int)jalDestino
    );

    fclose(f);

    // --- Llamada al traductor ---
    // Ya no saltamos directo (eso ejecutaba bytes Allegrex como si
    // fueran R5900 nativos, y funcionaba solo por casualidad con
    // instrucciones que coinciden entre ambos). Ahora el traductor
    // interpreta cada instruccion por software.

    u32 entryRelocalizado = header.e_entry + delta;

    if (header.e_entry < vaddrMinimo || header.e_entry >= vaddrMaximo) {
        printf("ERROR: el entry point (0x%08X) cae fuera del rango cargado.\n",
               (unsigned int)header.e_entry);
        while (1) {}
    }

    #define TAMANO_PILA 0x40000  // 256KB

    void *pilaReal = malloc(TAMANO_PILA);
    if (pilaReal == NULL) {
        printf("ERROR: no hay memoria para la pila.\n");
        while (1) {}
    }

    // SceModuleInfo vaddr = 0x003501BC (de la tabla de secciones del EBOOT)
    // gp_value esta al offset 0x20, module_start en offset 0x34.
    u8 *modInfo = (u8*)bloqueUnico + (0x003501BC - vaddrMinimo);
    u32 gpValue, moduleStart;
    memcpy(&gpValue, modInfo + 0x20, 4);
    memcpy(&moduleStart, modInfo + 0x34, 4);
    printf("gp_value (post-reloc) = 0x%08X\n", gpValue);
    printf("module_start (post-reloc) = 0x%08X\n", moduleStart);
    traductor_setRegistroInicial(28, gpValue); // $gp

    u32 pilaVirtualTope = (u32)pilaReal + TAMANO_PILA;
    traductor_setRegistroInicial(29, pilaVirtualTope);

    traductor_setRegistroInicial(31, PC_SALIDA_MODULO); // $ra

    traductor_setDelta(delta); // se sigue usando para el log, no para memoria

    traductor_setSegmentosArchivo(vaddrSegmentos, offsetSegmentos, fileszSegmentos, cantSegmentos);

    // rango REAL, no virtual
    traductor_setRangoVirtual((u32)bloqueUnico, (u32)bloqueUnico + tamanoTotal);

    // Crear un trampoline en BSS: JR $RA + NOP.
    // BSS empieza en vaddr 0x3F5780 (post-reloc queda en bloqueUnico + 0x3F5780).
    // El intérprete puede saltar ahí (está dentro del rango) y vuelve solo.
    u32 trampolineVaddr = 0x3F5780;
    u32 *pTrampoline = (u32 *)((u8 *)bloqueUnico + (trampolineVaddr - vaddrMinimo));
    pTrampoline[0] = 0x03E00008; // JR $RA
    pTrampoline[1] = 0x00000000; // NOP (delay slot)
    u32 trampolineReal = (u32)bloqueUnico + (trampolineVaddr - vaddrMinimo);
    printf("Trampoline GE en real=0x%08X\n", trampolineReal);

    // Parchear slots de vtable del GE en 0x302480 y 0x302484
    // (y vecinos que el juego lee con JALR $v1 en 0x4CDEE0)
    u32 *ptrGE0 = (u32 *)((u8 *)bloqueUnico + (0x172480 - vaddrMinimo));
    u32 *ptrGE1 = (u32 *)((u8 *)bloqueUnico + (0x172484 - vaddrMinimo));
    printf("Patch GE vtable: [0x302480]=0x%08X [0x302484]=0x%08X -> trampoline\n",
        *ptrGE0, *ptrGE1);
    *ptrGE0 = trampolineReal;
    *ptrGE1 = trampolineReal;

    // Directorio con el ISO/UMD extraido - RELATIVO a la raiz de "host:"
    // (que PCSX2 ya ancla en la carpeta compat_layer, ver el log:
    // "HLE Host: Set 'host:' root path to: ..."). Si tu carpeta con el
    // ISO extraido esta en otro lado, tiene que estar DENTRO de
    // compat_layer (como subcarpeta) o "host:" no la va a poder ver.
    // AJUSTAR "iso_extraido" al nombre real de tu subcarpeta.
    traductor_setDirectorioDatos("host:iso_extraido");

    printf("Arrancando el interprete en real=0x%08X...\n", (unsigned int)entryRelocalizado);
    traductor_ejecutar(entryRelocalizado);

    printf("El interprete termino o se detuvo.\n");
    while (1) {}

    return 0;
}