// traductor.c - Interprete de instrucciones Allegrex (MIPS de la PSP).
//
// FILOSOFIA: en vez de ejecutar los bytes del PRX como si fueran
// instrucciones nativas de la EE (lo que haciamos hasta ahora, y que
// funcionaba de pura casualidad con LW/ADDU porque coinciden con MIPS
// estandar), este modulo LEE cada instruccion, la DECODIFICA a mano,
// y EJECUTA su significado en C, usando un banco de registros propio
// (simulado), no los registros reales de la EE.
//
// Esto es mas lento que compilar a codigo nativo, pero muchisimo mas
// facil de extender: cuando aparezca una instruccion no implementada,
// el interprete va a avisar exactamente cual es y en que direccion,
// en vez de crashear la consola sin explicacion.

#include <tamtypes.h>
#include <stdio.h>
#include "traductor.h"
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <time.h>
#include <dirent.h>
#include <sys/stat.h>

#define TRADUCTOR_FIN_DATOS_ARCHIVO 0x003F5764

// ============================================================
// Acceso a memoria del modulo PSP
// ============================================================
// Todo el modulo ya fue relocalizado con un 'delta' fijo (ver
// compat_layer_loader.c). Para leer/escribir en una direccion
// virtual V del PSP original, la direccion real en la PS2 es
// simplemente V + delta. Guardamos ese delta como variable global
// del traductor (se setea una vez, al terminar la carga).


#define PSP_EXT_HEAP_BASE 0x6D510000
#define PSP_EXT_HEAP_SIZE 0x00100000  // 1MB
static u8 g_pspExtHeap[PSP_EXT_HEAP_SIZE];

#define VRAM_SIZE 0x00200000  // 2MB
static u8 g_vramBuffer[VRAM_SIZE] __attribute__((aligned(64)));

#define TRADUCTOR_MAX_THREADS 16
// Buffer estatico para la pila de cada hilo. Esto es un ELF MIPS de 32
// bits corriendo nativo (no un proceso de host de 64 bits), asi que su
// puntero real ya es una direccion PS2 valida - se usa directo, sin
// necesidad de mapearlo a una direccion ficticia.
//
// OJO: 64KB por hilo, NO 256KB. Un intento de subirlo a 256KB (4MB total
// de BSS extra) coincidio exactamente con el inicio de los crashes de
// PCSX2 - probablemente pisa algo del layout de memoria fijo que arma
// el loader. Si el juego pide mas de 64KB reales lo vamos a ver como
// desborde de pila (corrupcion silenciosa, no crash) y ahi lo resolvemos
// de otra forma (malloc dinamico contra el heap del juego, no BSS estatico).
#define TRADUCTOR_PILA_HILO_SLOT 0x00010000 // 64KB por hilo
// static u8 g_pilasSimuladas[TRADUCTOR_MAX_THREADS][TRADUCTOR_PILA_HILO_SLOT] __attribute__((aligned(64))); // unused

inline u8 *resolverDireccion(u32 addr) {
    if (addr >= PSP_EXT_HEAP_BASE &&
        addr < PSP_EXT_HEAP_BASE + PSP_EXT_HEAP_SIZE) {
        return g_pspExtHeap + (addr - PSP_EXT_HEAP_BASE);
    }
    return (u8 *)addr;
}

static s32 g_delta = 0;

static u32 g_vaddrSegmentos[TRADUCTOR_MAX_SEGMENTOS];
static u32 g_offsetSegmentos[TRADUCTOR_MAX_SEGMENTOS];
static u32 g_fileszSegmentos[TRADUCTOR_MAX_SEGMENTOS];
static int g_cantidadSegmentos = 0;

static u32 g_finDatosSegmentoMaximo = 0;

static u32 g_contadorInstrucciones = 0;

static u32 g_vaddrMinimo = 0;
static u32 g_vaddrMaximo = 0;

u64 g_fakeTick = 1000000ULL;

// static int g_loopDbg = 0;  // unused

static u8 *g_partitionPool = NULL;
static u32 g_partitionPoolSize = 0;
static u32 g_partitionPoolOffset = 0;

static u32 g_pcEjecutando = 0;

void traductor_setPartitionPool(u8 *pool, u32 size) {
    g_partitionPool = pool;
    g_partitionPoolSize = size;
    g_partitionPoolOffset = 0;
}

void traductor_setDelta(s32 delta) {
    g_delta = delta;
}

void traductor_setSegmentosArchivo(
    const u32 *vaddrSegmentos,
    const u32 *offsetSegmentos,
    const u32 *fileszSegmentos,
    int cantidadSegmentos
) {
    if (cantidadSegmentos < 0)
        cantidadSegmentos = 0;

    if (cantidadSegmentos > TRADUCTOR_MAX_SEGMENTOS)
        cantidadSegmentos = TRADUCTOR_MAX_SEGMENTOS;

    g_cantidadSegmentos = cantidadSegmentos;

    for (int i = 0; i < cantidadSegmentos; i++) {
        g_vaddrSegmentos[i] = vaddrSegmentos[i];
        g_offsetSegmentos[i] = offsetSegmentos[i];
        g_fileszSegmentos[i] = fileszSegmentos[i];

        u32 fin = vaddrSegmentos[i] + fileszSegmentos[i];

        if (fin > g_finDatosSegmentoMaximo)
            g_finDatosSegmentoMaximo = fin;
    }
}

void traductor_setRangoVirtual(u32 minimo, u32 maximo) {
    g_vaddrMinimo = minimo;
    g_vaddrMaximo = maximo;
}

// Permite fijar el valor inicial de un registro (en direccion VIRTUAL,
// si es una direccion de memoria) ANTES de arrancar el interprete.
// Se usa sobre todo para $sp (registro 29) - sin esto arranca en 0 y
// el modulo pisa su propio codigo apenas intenta guardar algo en la
// pila. Mas adelante puede servir tambien para $gp o argumentos.
static u32 g_registrosIniciales[32];
static int g_hayRegistrosIniciales[32];

void traductor_setRegistroInicial(int indice, u32 valor) {
    if (indice >= 0 && indice < 32) {
        g_registrosIniciales[indice] = valor;
        g_hayRegistrosIniciales[indice] = 1;
    }
}



// ============================================================
// Estado de la CPU simulada
// ============================================================

typedef struct {
    u32 gpr[32]; // gpr[0] siempre vale 0 (convencion MIPS) - lo forzamos despues de cada escritura
    u32 pc;      // direccion VIRTUAL (del PSP original) de la proxima instruccion
    u32 hi, lo;  // para multiplicacion/division
    u32 cop2_regs[128]; // VFPU real: 128 registros de 32 bits (8 bancos de matrices 4x4).
                        // Antes esto era [32] - alcanzaba de pura casualidad para
                        // MFC2/MTC2 (que solo usan el campo rd de 5 bits, 0-31),
                        // pero vmidt.q y cualquier instruccion de matriz de verdad
                        // necesitan el banco completo.
    int detenido;
} CpuSimulada;

static CpuSimulada cpu;

// Declaraciones adelantadas
static void ejecutarUnPaso(void);
extern int kjfs_resolverNid(u32 nid, u32 a0, u32 a1, u32 a2, u32 a3, u32 *resultado);

static float g_fpr[32];       // $f0 - $f31
static u32 g_fcr31;           // registro de control/status de la FPU

#define MAX_SEMAFOROS 32
static s32 g_semaCount[MAX_SEMAFOROS];
static s32 g_semaSiguiente = 1;

// ---- Message pipes (sceKernel*MsgPipe) ----
// Cola interna no acotada por simplicidad: SendMsgPipe SIEMPRE encola (nunca
// se bloquea el emisor en un pipe lleno), y cede el CPU para que el hilo
// worker/receptor pueda correr. ReceiveMsgPipe se bloquea (HILO_ESPERANDO_PIPE)
// hasta que haya un mensaje; lo despierta un SendMsgPipe que encola. Esto
// imita el handshake productor/consumidor de la PSP sin necesidad de re-ejecutar
// el syscall al despertar (el recurso compartido es la cola, no un contador).
typedef struct MsgNode {
    u8 *data;
    u32 size;
    struct MsgNode *next;
} MsgNode;

typedef struct {
    u32 id;
    int used;
    MsgNode *head;
    MsgNode *tail;
    int waitRecv[TRADUCTOR_MAX_THREADS]; // hilos bloqueados haciendo ReceiveMsgPipe
    int nWaitRecv;
} MsgPipe;

#define MAX_MSGPIPES 16
static MsgPipe g_msgpipes[MAX_MSGPIPES];
static u32 g_msgpipeSiguiente = 1;

static int msgpipe_buscarPorId(u32 id) {
    for (int i = 0; i < MAX_MSGPIPES; i++)
        if (g_msgpipes[i].used && g_msgpipes[i].id == id) return i;
    return -1;
}

// Nombres de registros, solo para los logs de diagnostico
static const char *nombreReg[32] = {
    "zero","at","v0","v1","a0","a1","a2","a3",
    "t0","t1","t2","t3","t4","t5","t6","t7",
    "s0","s1","s2","s3","s4","s5","s6","s7",
    "t8","t9","k0","k1","gp","sp","fp","ra"
};

static inline u32 memLeer32(u32 addr)  { return *(u32 *)resolverDireccion(addr); }
static inline void memEscribir32(u32 addr, u32 v) {
    if (addr == 0x00302484) {
        printf("WRITE 0x302484 = 0x%08X (pc=0x%08X paso=%u)\n",
            v, (unsigned int)g_pcEjecutando,
            (unsigned int)g_contadorInstrucciones);
    }
    *(u32 *)resolverDireccion(addr) = v;
}
static inline u16 memLeer16(u32 addr)  { return *(u16 *)resolverDireccion(addr); }
static inline void memEscribir16(u32 addr, u16 v) { *(u16 *)resolverDireccion(addr) = v; }
static inline u8  memLeer8(u32 addr)   { return *(u8  *)resolverDireccion(addr); }
static inline void memEscribir8(u32 addr, u8 v)   { *(u8  *)resolverDireccion(addr) = v; }


// ============================================================
// Decodificacion
// ============================================================
// Campos estandar de una instruccion MIPS de 32 bits:
//   opcode(6) rs(5) rt(5) rd(5) shamt(5) funct(6)   <- tipo R
//   opcode(6) rs(5) rt(5) inmediato(16)              <- tipo I
//   opcode(6) direccion(26)                          <- tipo J

#define OPCODE(i)   (((i) >> 26) & 0x3F)
#define RS(i)       (((i) >> 21) & 0x1F)
#define RT(i)       (((i) >> 16) & 0x1F)
#define RD(i)       (((i) >> 11) & 0x1F)
#define SHAMT(i)    (((i) >> 6)  & 0x1F)
#define FUNCT(i)    ((i) & 0x3F)
#define IMM16(i)    ((i) & 0xFFFF)
#define IMM16_SEXT(i) ((s32)(s16)((i) & 0xFFFF))  // inmediato con signo, extendido a 32 bits
#define DIR26(i)    ((i) & 0x3FFFFFF)

static inline void escribirGpr(int indice, u32 valor) {
    if (indice != 0) {
        cpu.gpr[indice] = valor;
    }
    // gpr[0] se ignora silenciosamente: en MIPS $zero siempre es 0,
    // el compilador a veces lo usa como destino "basurero".
}

static int traductor_vaddrAOffsetArchivo(u32 direccionReal, u32 *offsetArchivo) {
    u32 vaddr = direccionReal - (u32)g_delta;
    for (int i = 0; i < g_cantidadSegmentos; i++) {
        u32 inicio = g_vaddrSegmentos[i];
        u32 fin = inicio + g_fileszSegmentos[i];
        if (vaddr >= inicio && vaddr < fin) {
            *offsetArchivo = g_offsetSegmentos[i] + (vaddr - inicio);
            return 1;
        }
    }
    return 0;
}

// Traduce un registro VFPU de 7 bits + (columna, fila) logicas a un indice
// fisico dentro de cpu.cop2_regs[128]. El bit de "transpuesta" (bit 5 del
// registro) intercambia como se interpreta la grilla fisica del banco -
// mismo esquema para vectores fila/columna y para matrices completas.
static inline u32 vfpuIdx(u32 regval, u32 col, u32 fila) {
    u32 banco = (regval >> 2) & 7;
    u32 transpuesta = (regval >> 5) & 1;
    return transpuesta ? (banco*32 + fila*4 + col) : (banco*32 + col*4 + fila);
}

// Registro VFPU "single" (un solo float, no vector): a diferencia de los
// vectores/matrices, aca los 7 bits del registro codifican directamente
// banco+columna+fila sin ambiguedad de transpuesta (bank=(reg>>2)&7,
// col=reg&3, fila=(reg>>5)&3 - usa los 2 bits altos completos, no 1 solo).
static inline u32 vfpuSingleIdx(u32 regval) {
    u32 banco = (regval >> 2) & 7;
    u32 col = regval & 3;
    u32 fila = (regval >> 5) & 3;
    return banco*32 + col*4 + fila;
}

// ============================================================
// Prefijos VFPU (vpfxs/vpfxt/vpfxd)
// ============================================================
// Modifican como la SIGUIENTE instruccion VFPU (una sola vez, despues
// se apagan solos) lee sus fuentes (swizzle/negacion/valor absoluto,
// o una constante fija) o escribe su destino (mascara de escritura,
// saturacion). Si no se implementan, cualquier instruccion VFPU que
// ya "funciona" puede dar resultados mal silenciosamente en cuanto el
// juego use un prefijo - no tira error, simplemente calcula cualquier
// cosa.
typedef struct {
    int activo;
    u32 swizzle[4];
    int negar[4];
    int abs_[4];
    int constante[4]; // si esta seteado: NO soportado (raro), se avisa
} PrefijoFuente;

typedef struct {
    int activo;
    int escribir[4]; // 1 = escribir este componente, 0 = mascarado (no tocar)
    int saturar[4];  // 0=nada, 1=clamp[0,1], 3=clamp[-1,1] (2 no se usa)
} PrefijoDestino;

static PrefijoFuente g_pfxS = {0};
static PrefijoFuente g_pfxT = {0};
static PrefijoDestino g_pfxD = {0};

static void vfpuParsearPrefijoFuente(u32 instr, PrefijoFuente *pfx) {
    pfx->activo = 1;
    for (int c = 0; c < 4; c++) {
        pfx->swizzle[c]   = (instr >> (c * 2)) & 0x3;
        pfx->abs_[c]      = (instr >> (8 + c)) & 0x1;
        pfx->constante[c] = (instr >> (12 + c)) & 0x1;
        pfx->negar[c]     = (instr >> (16 + c)) & 0x1;
    }
}

static void vfpuParsearPrefijoDestino(u32 instr, PrefijoDestino *pfx) {
    pfx->activo = 1;
    for (int c = 0; c < 4; c++) {
        pfx->saturar[c]  = (instr >> (c * 2)) & 0x3;
        pfx->escribir[c] = ((instr >> (8 + c)) & 0x1) ? 0 : 1; // bit=1 significa "mascarado"
    }
}

// ============================================================
// Sub-interrupciones registradas (sceKernelRegisterSubIntrHandler)
// ============================================================
#define TRADUCTOR_MAX_SUBINTR 16
typedef struct {
    u32 subIntr;
    u32 handler;
    u32 common;
    int habilitado;
} InfoSubIntr;

static InfoSubIntr g_subIntr[TRADUCTOR_MAX_SUBINTR];
static int g_cantidadSubIntr = 0;

static int g_enInvocacionCallback = 0; // no dejar que un import faltante DENTRO de un callback mate todo, ni que el scheduler cambie de hilo a mitad de una invocacion

static void logNoImplementado(u32 instr, u32 pcInstr, const char *motivo) {
    u32 direccionReal = pcInstr;

    u32 offsetArchivo = 0;
    int tieneOffsetArchivo =
        traductor_vaddrAOffsetArchivo(pcInstr, &offsetArchivo);

    printf(
        "TRADUCTOR: instruccion no implementada (%s)\n"
        "  instr       = 0x%08X\n"
        "  vaddr       = 0x%08X\n"
        "  ram         = 0x%08X\n"
        "  opcode      = 0x%02X\n"
        "  funct       = 0x%02X\n"
        "  rs          = %s\n"
        "  rt          = %s\n"
        "  rd          = %s\n"
        "  shamt       = 0x%02X (%u)\n"
        "  imm         = 0x%04X\n",
        motivo,
        (unsigned int)instr,
        (unsigned int)pcInstr,
        (unsigned int)direccionReal,
        (unsigned int)OPCODE(instr),
        (unsigned int)FUNCT(instr),
        nombreReg[RS(instr)],
        nombreReg[RT(instr)],
        nombreReg[RD(instr)],
        (unsigned int)SHAMT(instr),
        (unsigned int)SHAMT(instr),
        (unsigned int)IMM16(instr)
    );

    if (tieneOffsetArchivo) {
        printf(
            "  EBOOT.BIN offset = 0x%08X\n",
            (unsigned int)offsetArchivo
        );
    } else {
        printf(
            "  EBOOT.BIN offset = DESCONOCIDO "
            "(vaddr fuera de los bytes cargados de los PT_LOAD)\n"
        );
    }

    cpu.detenido = 1;
}

// Calcula el offset de arranque (para .p/.t que pueden empezar a mitad
// de fila/columna) - mismo criterio usado en todos los case VFPU.
static inline u32 vfpuOffsetTamano(u32 regval, u32 elementos) {
    if (elementos == 2) return ((regval >> 6) & 1) ? 2 : 0;
    if (elementos == 3) return ((regval >> 6) & 1) ? 1 : 0;
    return 0;
}

// Lee el elemento logico "i" (0..elementos-1) de un operando FUENTE,
// aplicando swizzle/abs/negacion si hay un prefijo pendiente para esa
// fuente. pfx puede ser NULL (sin prefijo, lectura directa).
static float vfpuLeerFuente(u32 reg, u32 elementos, u32 i, PrefijoFuente *pfx) {
    u32 idxLeer = i;
    int aplicarAbs = 0, aplicarNeg = 0;

    if (pfx && pfx->activo) {
        if (pfx->constante[i]) {
            logNoImplementado(0, 0, "VFPU prefijo con componente CONSTANTE (%vk) no soportado - resultado incorrecto");
        }
        idxLeer = pfx->swizzle[i];
        if (idxLeer >= elementos) idxLeer = elementos - 1; // defensivo
        aplicarAbs = pfx->abs_[i];
        aplicarNeg = pfx->negar[i];
    }

    float val;
    if (elementos == 1) {
        memcpy(&val, &cpu.cop2_regs[vfpuSingleIdx(reg)], 4);
    } else {
        u32 off = vfpuOffsetTamano(reg, elementos);
        memcpy(&val, &cpu.cop2_regs[vfpuIdx(reg, reg & 3, off + idxLeer)], 4);
    }
    if (aplicarAbs) val = fabsf(val);
    if (aplicarNeg) val = -val;
    return val;
}

// Escribe el elemento logico "i" de un operando DESTINO, respetando
// mascara de escritura y saturacion si hay un prefijo vpfxd pendiente.
// Devuelve 1 si de verdad escribio, 0 si el componente estaba mascarado.
static int vfpuEscribirDestino(u32 reg, u32 elementos, u32 i, float val, PrefijoDestino *pfx) {
    if (pfx && pfx->activo) {
        if (!pfx->escribir[i]) return 0; // componente mascarado, no tocar
        if (pfx->saturar[i] == 1) val = val < 0.0f ? 0.0f : (val > 1.0f ? 1.0f : val);
        else if (pfx->saturar[i] == 3) val = val < -1.0f ? -1.0f : (val > 1.0f ? 1.0f : val);
    }
    if (elementos == 1) {
        memcpy(&cpu.cop2_regs[vfpuSingleIdx(reg)], &val, 4);
    } else {
        u32 off = vfpuOffsetTamano(reg, elementos);
        memcpy(&cpu.cop2_regs[vfpuIdx(reg, reg & 3, off + i)], &val, 4);
    }
    return 1;
}

// Los prefijos son de un solo uso: la PROXIMA instruccion VFPU que
// lea/escriba vectores los consume y los apaga, sin importar si esa
// instruccion estaba implementada o no (asi se comporta el hardware).
static void vfpuConsumirPrefijos(void) {
    g_pfxS.activo = 0;
    g_pfxT.activo = 0;
    g_pfxD.activo = 0;
}

u32 g_importNid[TRADUCTOR_MAX_IMPORTS];
const char *g_importNombreLib[TRADUCTOR_MAX_IMPORTS];
u32 g_cantidadImportsRegistrados = 0;

u32 traductor_registrarImport(u32 indice, u32 nid, const char *nombreLibreria) {
    if (indice < TRADUCTOR_MAX_IMPORTS) {
        g_importNid[indice] = nid;
        g_importNombreLib[indice] = nombreLibreria;
        if (indice + 1 > g_cantidadImportsRegistrados)
            g_cantidadImportsRegistrados = indice + 1;
        return indice;
    }
    return 0xFFFFFFFF;
}

// ============================================================
// Modulos PSP adicionales (kjfs, etc.)
// ============================================================
// Cada modulo cargado vive en su propio bloque de memoria REAL, con su
// propio $gp. Los exports se resuelven por NID: cuando un import coincide
// con un export registrado, el interprete salta al codigo real del modulo
// (haciendo el swap de $gp que exige cruzar de modulo) en vez de ejecutar
// una funcion C de compatibilidad.
typedef struct {
    int activo;
    u32 realMin, realMax;   // rango REAL (post-reloc) del codigo/datos
    u32 vaddrMin, vaddrMax; // rango VIRTUAL original del modulo (para mapear JR/JALR)
    u32 gpValue;            // $gp REAL del modulo
    u32 moduleStart;        // direccion REAL de module_start (0 si no tiene)
    int tieneModuleStart;
} ModuloPSP;

#define MAX_MODULOS 8
static ModuloPSP g_modulos[MAX_MODULOS];
static int g_cantidadModulos = 0;

typedef struct {
    u32 nid;
    u32 addrReal;   // direccion REAL de la funcion exportada
    int modulo;     // indice en g_modulos
} ExportPSP;

#define MAX_EXPORTS 2048
static ExportPSP g_exports[MAX_EXPORTS];
static int g_cantidadExports = 0;

// Pila de frames para el swap de $gp al cruzar de modulo (soporta anidacion:
// EBOOT->kjfs->EBOOT->kjfs...). Cada hijack empuja (gp del llamador, addr de
// retorno); el return sentinel los restaura.
typedef struct {
    u32 caller_gp;
    u32 return_addr;
} FrameModulo;

#define MAX_PILA_MODULO 32
static FrameModulo g_pilaModulo[MAX_PILA_MODULO];
static int g_spModulo = 0;

int traductor_registrarModulo(u32 realMin, u32 realMax, u32 vaddrMin, u32 vaddrMax, u32 gpValue,
                              u32 moduleStart, int tieneModuleStart) {
    if (g_cantidadModulos >= MAX_MODULOS) {
        printf("TRADUCTOR: no hay modulo libre para registrar (max=%d)\n", MAX_MODULOS);
        return -1;
    }
    int idx = g_cantidadModulos++;
    g_modulos[idx].activo = 1;
    g_modulos[idx].realMin = realMin;
    g_modulos[idx].realMax = realMax;
    g_modulos[idx].vaddrMin = vaddrMin;
    g_modulos[idx].vaddrMax = vaddrMax;
    g_modulos[idx].gpValue = gpValue;
    g_modulos[idx].moduleStart = moduleStart;
    g_modulos[idx].tieneModuleStart = tieneModuleStart;
    printf("TRADUCTOR: modulo %d registrado real=0x%08X..0x%08X vaddr=0x%08X..0x%08X gp=0x%08X moduleStart=0x%08X\n",
           idx, (unsigned int)realMin, (unsigned int)realMax, (unsigned int)vaddrMin, (unsigned int)vaddrMax,
           (unsigned int)gpValue, (unsigned int)moduleStart);
    return idx;
}

void traductor_registrarExport(u32 nid, u32 addrReal, int moduloIdx) {
    if (g_cantidadExports >= MAX_EXPORTS) {
        printf("TRADUCTOR: tabla de exports llena (max=%d)\n", MAX_EXPORTS);
        return;
    }
    g_exports[g_cantidadExports].nid = nid;
    g_exports[g_cantidadExports].addrReal = addrReal;
    g_exports[g_cantidadExports].modulo = moduloIdx;
    g_cantidadExports++;
}

// Devuelve el indice del export con ese NID, o -1.
static int buscarExport(u32 nid) {
    for (int i = 0; i < g_cantidadExports; i++) {
        if (g_exports[i].nid == nid) return i;
    }
    return -1;
}

// Ejecuta el module_start del modulo (si lo tiene). Se usa para inicializar
// el modulo (globales, tablas internas) antes de que el juego lo use.
void traductor_iniciarModulo(int idx) {
    if (idx < 0 || idx >= g_cantidadModulos || !g_modulos[idx].activo) return;
    ModuloPSP *m = &g_modulos[idx];
    if (!m->tieneModuleStart || m->moduleStart == 0) return;

    static u8 pilaModuloStart[0x40000] __attribute__((aligned(64)));

    CpuSimulada guardado = cpu;

    cpu.gpr[28] = m->gpValue;                              // $gp del modulo
    cpu.gpr[29] = (u32)(pilaModuloStart + sizeof(pilaModuloStart) - 64); // $sp
    cpu.gpr[31] = PC_CALLBACK_SALIDA;                     // $ra = sentinel
    cpu.gpr[4]  = 0;  // a0 (argc)
    cpu.gpr[5]  = 0;  // a1 (argp)
    cpu.gpr[6]  = 0;  // a2
    cpu.gpr[7]  = 0;  // a3
    cpu.pc = m->moduleStart;
    cpu.hi = cpu.lo = 0;
    memset(cpu.cop2_regs, 0, sizeof(cpu.cop2_regs));

    int enInvAnt = g_enInvocacionCallback;
    g_enInvocacionCallback = 1;

    const int MAX_PASOS = 4000000;
    int pasos = 0;
    while (cpu.pc != PC_CALLBACK_SALIDA && !cpu.detenido && pasos < MAX_PASOS) {
        ejecutarUnPaso();
        pasos++;
    }

    if (pasos >= MAX_PASOS) {
        printf("TRADUCTOR: module_start del modulo %d no volvio (limite)\n", idx);
    } else {
        printf("TRADUCTOR: module_start del modulo %d ejecutado OK (%d pasos)\n", idx, pasos);
    }

    g_enInvocacionCallback = enInvAnt;
    cpu = guardado; // restaurar contexto del llamador (el loader principal)
}


// ============================================================
// Hilos - scheduler cooperativo REAL
// ============================================================
// Cada hilo tiene su PROPIO banco de registros completo (CpuSimulada),
// no comparte nada con los demas salvo $gp (convencion MIPS/ABI, todos
// los hilos de un mismo modulo apuntan a la misma tabla de globales).
// El "cpu" global de siempre pasa a ser, en todo momento, una COPIA de
// trabajo del hilo que tiene el CPU ahora (g_hiloActual). Al cambiar
// de hilo: 1) el hilo saliente guarda "cpu" en su slot, 2) el hilo
// entrante carga su slot en "cpu".
//
// Es cooperativo, no preemptivo de verdad: solo cambia de hilo en
// puntos de cesion explicitos (el hilo actual termina, se duerme, o
// se bloquea esperando un semaforo). Si un hilo nunca cede, los demas
// no corren nunca - igual que pasaria en la PSP real si ese hilo
// tuviera prioridad mas alta y nunca llamara a nada que ceda CPU.


Hilo g_hilos[TRADUCTOR_MAX_THREADS];
int g_cantidadHilos = 0;
int g_hiloActual = 0;

// CpuSimulada for each thread (separate from Hilo struct to avoid header conflicts)
CpuSimulada g_hilos_cpu[TRADUCTOR_MAX_THREADS];

#define TRADUCTOR_MAX_CALLBACKS 16

typedef struct {
    u32 uid;
    u32 func;
    u32 arg;
} InfoCallback;

static InfoCallback g_callbacks[TRADUCTOR_MAX_CALLBACKS];
static u32 g_cantidadCallbacks = 0;

// Cuando un import cambia cpu.pc el mismo (hijack de salto), esta
// bandera le avisa al bucle principal que NO pise ese pc con el
// destino normal del salto (el jr $ra del stub).
static int g_pcForzadoPorImport = 0;

// SendMsgPipe encola un mensaje y quiere ceder el CPU para que el hilo
// worker/receptor corra. No puede llamar planificar() adentro (pisaria
// $v0 del llamador), asi que prende esta bandera y ejecutarUnPaso la
// usa para forzar una cesion cooperativa justo antes de devolver.
static int g_forzarCesion = 0;

static void hilo_guardarActual(void) {
    g_hilos_cpu[g_hiloActual] = cpu;
}

static int hilo_buscarSiguienteListo(void) {
    for (int i = 1; i <= g_cantidadHilos; i++) {
        int idx = (g_hiloActual + i) % g_cantidadHilos;
        if (g_hilos[idx].estado == HILO_LISTO) return idx;
    }
    return -1;
}

// Guarda el hilo actual y le pasa el CPU a otro hilo LISTO (round robin).
// Devuelve 0 si no queda NINGUN hilo listo (deadlock real: todos
// dormidos o bloqueados en semaforos que nadie va a señalar).
static int planificar(void) {
    u32 uidSaliente = g_hilos[g_hiloActual].uid;
    
    // Solo guardamos si el hilo actual sigue dentro del rango y no termino
    if (g_hiloActual >= 0 && g_hiloActual < g_cantidadHilos) {
        hilo_guardarActual();
    }

    int siguiente = hilo_buscarSiguienteListo();
    if (siguiente == -1) {
        if (g_hiloActual >= 0 && g_hiloActual < g_cantidadHilos && g_hilos[g_hiloActual].estado == HILO_LISTO) {
            cpu = g_hilos_cpu[g_hiloActual];
            return 1;
        }
        printf("TRADUCTOR: DEADLOCK - ningun hilo listo\n");
        return 0;
    }

    g_hiloActual = siguiente;
    cpu = g_hilos_cpu[siguiente];

    printf("TRADUCTOR: CAMBIO DE CONTEXTO uid_sale=0x%08X uid_entra=0x%08X "
           "pc=0x%08X sp=0x%08X ra=0x%08X gp=0x%08X estado=%d\n",
        (unsigned int)uidSaliente,
        (unsigned int)g_hilos[siguiente].uid,
        (unsigned int)cpu.pc,
        (unsigned int)cpu.gpr[29],
        (unsigned int)cpu.gpr[31],
        (unsigned int)cpu.gpr[28],
        (int)g_hilos[siguiente].estado);
    return 1;
}

// Contador para el rastreo detallado de las primeras instrucciones de
// CUALQUIER hilo que no sea el principal - para ver exactamente que
// pasa apenas arranca a correr codigo real de un hilo nuevo.
static int g_trazaHiloNuevo = 0;

// Un SignalSema puede haber desbloqueado hilos en HILO_ESPERANDO_SEMA.
static void hilo_revisarEsperasDeSemaforo(u32 semaId) {
    for (int i = 0; i < g_cantidadHilos; i++) {
        if (g_hilos[i].estado == HILO_ESPERANDO_SEMA &&
            g_hilos[i].semaEsperado == semaId &&
            semaId < MAX_SEMAFOROS &&
            g_semaCount[semaId] >= g_hilos[i].semaCantidad) {
            g_semaCount[semaId] -= g_hilos[i].semaCantidad;
            g_hilos[i].estado = HILO_LISTO;
        }
    }
}

#define TRADUCTOR_MAX_BLOQUES_MEM 32

typedef struct {
    u32 uid;
    u32 addr;
    u32 size;
} BloqueMemoria;

static BloqueMemoria g_bloquesMem[TRADUCTOR_MAX_BLOQUES_MEM];
static u32 g_cantidadBloques = 0;

// ============================================================
// sceIo* reales - redirigir rutas PSP a archivos de verdad dentro
// del ISO/UMD extraido en el host.
// ============================================================

#define TRADUCTOR_MAX_ARCHIVOS 32
#define TRADUCTOR_DIR_MAXLEN 200

// Manejador de un "archivo" abierto. Puede ser un FILE* comun (para archivos
// reales) o un DIR* (para cuando se abre la raiz de un device o se usa
// sceIoDopen). kjfs abre "fatms0:" (la raiz del memory stick, que es un
// directorio) y necesita que eso devuelva un fd valido.
char g_directorioDatos[TRADUCTOR_DIR_MAXLEN] = "";
ArchivoAbierto g_archivosAbiertos[TRADUCTOR_MAX_ARCHIVOS];

void traductor_setDirectorioDatos(const char *ruta) {
    if (!ruta) return;
    size_t len = strlen(ruta);
    if (len >= TRADUCTOR_DIR_MAXLEN) len = TRADUCTOR_DIR_MAXLEN - 1;
    memcpy(g_directorioDatos, ruta, len);
    g_directorioDatos[len] = '\0';
}

// Traduce una ruta PSP (ej: "disc0:/PSP_GAME/USRDIR/data.bin",
// "fatms0:/algo", "umd0:/otro") a una ruta real dentro de
// g_directorioDatos. Saca cualquier prefijo "algo:" que tenga (device
// de PSP), y el "/" que le siga si lo hay.
void traducirRutaPSP(const char *rutaPsp, char *salida, size_t salidaMax) {
    const char *resto = rutaPsp;
    const char *dosPuntos = strchr(rutaPsp, ':');
    if (dosPuntos) {
        resto = dosPuntos + 1;
        if (*resto == '/') resto++;
    }
    snprintf(salida, salidaMax, "%s/%s", g_directorioDatos, resto);
}

// ============================================================
// Interprete de listas de comandos de la GE (Graphics Engine)
// ============================================================
// Cada comando de una lista GE es una word de 32 bits: los 8 bits
// altos son el comando, los 24 bits bajos son el argumento. Las
// direcciones (JUMP/CALL/BASE) se manejan igual que en todo el resto
// del interprete: los punteros que pone el JUEGO en tiempo de
// ejecucion ya son direcciones reales (post-relocalizacion), no hace
// falta sumarles delta de nuevo aca.
//
// PRIMER PASO: solo recorrer la lista completa y contar que comandos
// aparecen (histograma), sin dibujar nada todavia. Sirve para saber,
// con datos reales de ESTE juego, que hace falta implementar despues
// (PRIM con que formato de vertice, si usa texturas, etc) en vez de
// adivinar.

#define GE_CMD_NOP     0x00
#define GE_CMD_VADDR   0x01
#define GE_CMD_IADDR   0x02
#define GE_CMD_PRIM    0x04
#define GE_CMD_JUMP    0x08
#define GE_CMD_BJUMP   0x09
#define GE_CMD_CALL    0x0A
#define GE_CMD_RET     0x0B
#define GE_CMD_END     0x0C
#define GE_CMD_SIGNAL  0x0E
#define GE_CMD_FINISH  0x0F
#define GE_CMD_BASE    0x10

static u32 g_geHistograma[256];
static u32 g_geBase = 0;

static void geProcesarLista(u32 direccionLista, int mostrarDetalle) {
    u32 pc = direccionLista;
    u32 pilaRetorno[32];
    int profundidadPila = 0;
    int pasos = 0;
    const int MAX_PASOS = 200000; // salvavidas por si END/FINISH nunca aparece

    g_geBase = 0;
    memset(g_geHistograma, 0, sizeof(g_geHistograma)); // limpiar del llamado anterior

    while (pasos < MAX_PASOS) {
        pasos++;
        u32 word = memLeer32(pc);
        u32 cmd = (word >> 24) & 0xFF;
        u32 arg = word & 0xFFFFFF;

        g_geHistograma[cmd]++;

        if (cmd == GE_CMD_BASE) {
            g_geBase = (arg & 0xFF) << 24;
        } else if (cmd == GE_CMD_JUMP || cmd == GE_CMD_BJUMP) {
            pc = ((g_geBase | arg) & 0x1FFFFFFF) - 4; // -4 porque el while suma 4 al final
        } else if (cmd == GE_CMD_CALL) {
            if (profundidadPila < 32) pilaRetorno[profundidadPila++] = pc + 4;
            pc = ((g_geBase | arg) & 0x1FFFFFFF) - 4;
        } else if (cmd == GE_CMD_RET) {
            if (profundidadPila > 0) pc = pilaRetorno[--profundidadPila] - 4;
        } else if (cmd == GE_CMD_END || cmd == GE_CMD_FINISH) {
            if (mostrarDetalle) {
                printf("TRADUCTOR GE: lista terminada (cmd=0x%02X) despues de %d comandos\n",
                    (unsigned int)cmd, pasos);
            }
            return;
        }
        // Cualquier otro comando: por ahora solo se cuenta en el
        // histograma, no se ejecuta de verdad. Ese es el proximo paso.

        pc += 4;
    }

    if (mostrarDetalle) {
        printf("TRADUCTOR GE: lista cortada por limite de seguridad (%d pasos) - "
               "puede que falte manejar bien algun JUMP/CALL, o que la lista "
               "de verdad no tenga END/FINISH\n", pasos);
    }
}

static void geImprimirHistograma(void) {
    printf("TRADUCTOR GE: comandos vistos en la lista:\n");
    for (int i = 0; i < 256; i++) {
        if (g_geHistograma[i] > 0) {
            printf("  cmd=0x%02X count=%u\n", i, (unsigned int)g_geHistograma[i]);
        }
    }
}

u32 resolverImport(u32 indice, u32 a0, u32 a1, u32 a2, u32 a3, int *implementado) {
    (void)a0; (void)a1; (void)a2; (void)a3;
    *implementado = 1;

    // Importante: resetear el flag por llamada. Cuando entramos a un modulo
    // (kjfs) via hijack este flag queda en 1, pero mientras ejecutamos el
    // codigo REAL del modulo este puede llamar NIDs que SI estan implementados
    // en C (nids_kjfs.c o el switch principal). Para esos, el bucle DEBE
    // avanzar pc y el handler SYSCALL DEBE escribir $v0. El hijack lo re-seteara
    // a 1 solo para su propia resolucion.
    g_pcForzadoPorImport = 0;

    u32 nid = (indice < g_cantidadImportsRegistrados) ? g_importNid[indice] : 0;

    // DEBUG: log cada NID de kjfs que se llama (via hijack o HLE), y si
    // ademas es un export de kjfs que caeria al hijack sin handler HLE.
    if (indice < g_cantidadImportsRegistrados &&
        g_importNombreLib[indice] &&
        strcmp(g_importNombreLib[indice], "kjfs") == 0) {
        int esEx = buscarExport(nid);
        printf("KJFS_IMPORT: nid=0x%08X a0=0x%08X a1=0x%08X a2=0x%08X a3=0x%08X %s\n",
               (unsigned int)nid, (unsigned int)a0, (unsigned int)a1, (unsigned int)a2, (unsigned int)a3,
               (esEx >= 0) ? "<== EXPORT KJFS (hijack)" : "");
    }

    // --- NIDs especificos de kjfs (nids_kjfs.c) ---
    // Se intenta PRIMERO que el hijack: si el EBOOT llama un export de kjfs
    // cuyo NID tenemos implementado en C (HLE), usamos el HLE en vez de
    // saltar al codigo real no inicializado de kjfs (module_start=0).
    {
        u32 resKjfs = 0;
        if (kjfs_resolverNid(nid, a0, a1, a2, a3, &resKjfs)) {
            return resKjfs;
        }
    }

    // --- Ruta a modulo (hijack de salto) ---
    // Si este NID es un export registrado de algun modulo cargado (p.ej. una
    // funcion de kjfs), en vez de ejecutar una funcion C de compatibilidad
    // saltamos al codigo REAL del modulo. Hacemos el swap de $gp (cada modulo
    // tiene el suyo) y dejamos $ra apuntando a un sentinel de retorno; al
    // volver, el bucle restaura el $gp del llamador y retorna a quien llamo.
    int ex = buscarExport(nid);
    if (ex >= 0) {
        ModuloPSP *m = &g_modulos[g_exports[ex].modulo];
        if (g_spModulo < MAX_PILA_MODULO) {
            g_pilaModulo[g_spModulo].caller_gp   = cpu.gpr[28];
            g_pilaModulo[g_spModulo].return_addr = cpu.gpr[31];
            g_spModulo++;
        } else {
            printf("TRADUCTOR: pila de frames de modulo llena (anidacion excesiva) nid=0x%08X\n",
                   (unsigned int)nid);
        }
        cpu.gpr[28] = m->gpValue;   // $gp del modulo llamado
        cpu.gpr[31] = PC_MODULO_RET; // $ra = sentinel de retorno
        cpu.pc      = g_exports[ex].addrReal;
        g_pcForzadoPorImport = 1;   // el bucle no debe pisar pc ni escribir $v0
        return 0;
    }

    if (indice < g_cantidadImportsRegistrados &&
        g_importNombreLib[indice] &&
        strcmp(g_importNombreLib[indice], "SysMemUserForUser") == 0) {
        printf("SYSMEM nid=0x%08X a0=0x%08X a1=0x%08X a2=0x%08X a3=0x%08X\n",
               (unsigned int)g_importNid[indice], a0, a1, a2, a3);
    }

    *implementado = 1;
    switch(nid) {
        case 0x91DE343C: // sceKernelSetCompiledSdkVersion500_505
            return 0;
        case 0xF77D77CB: // sceKernelSetCompilerVersion
            return 0;

        case 0x446D8DE6: { // sceKernelCreateThread(name, entry, prio, stackSize, attr, option)
            if (g_cantidadHilos >= TRADUCTOR_MAX_THREADS) {
                printf("TRADUCTOR: limite de hilos alcanzado\n");
                return 0x80020001;
            }
            u32 uid = 0x10000 + g_cantidadHilos;
            Hilo *h = &g_hilos[g_cantidadHilos];
            h->uid = uid;
            h->entry = a1;
            h->stackSize = a3 ? a3 : 0x10000;
            h->estado = HILO_CREADO;
            h->wakeupsPendientes = 0;
            g_cantidadHilos++;
            printf("TRADUCTOR: CreateThread entry=0x%08X stack=0x%08X uid=0x%08X\n",
                (unsigned int)a1, (unsigned int)a3, (unsigned int)uid);
            return uid;
        }

        case 0xF475845D: { // sceKernelStartThread(thid, arglen, argp)
                u32 thid = a0;
                int idx = -1;
                for (int i = 0; i < g_cantidadHilos; i++) {
                    if (g_hilos[i].uid == thid) { idx = i; break; }
                }
                if (idx == -1) return 0x80020001;

                Hilo *h = &g_hilos[idx];
                CpuSimulada *hc = &g_hilos_cpu[idx];

                for (int r = 0; r < 32; r++) hc->gpr[r] = 0;
                hc->gpr[28] = cpu.gpr[28]; // $gp del creador

                u32 tamanoPila = (h->stackSize > 0 && h->stackSize <= TRADUCTOR_PILA_HILO_SLOT)
                                 ? h->stackSize
                                 : TRADUCTOR_PILA_HILO_SLOT;

                // Mapeamos la pila en una zona segura dentro de los 32MB de RAM de PS2 (EE RAM)
                // Usamos un offset fijo dentro de 0x01000000 (16MB a 32MB)
                u32 basePila = 0x01000000 + (idx * TRADUCTOR_PILA_HILO_SLOT);

                // Calculamos el TOP de la pila y forzamos MÁSCARA 0x01FFFFFF
                // Esto remueve CUALQUIER bit Kernel/Uncached (0x80000000 / 0xA0000000)
                // para que PCSX2 lo mapee dentro de la RAM virtual sin desbordar.
                u32 spCalculado = ((basePila + tamanoPila) - 64) & 0x01FFFFC0;

                hc->gpr[29] = spCalculado; // $sp

                hc->gpr[4]  = a1; // $a0 = arglen
                hc->gpr[5]  = a2; // $a1 = argp
                hc->gpr[31] = PC_HILO_SALIDA;
                hc->pc      = h->entry;
                hc->hi = hc->lo = 0;
                hc->detenido = 0;
                memset(hc->cop2_regs, 0, sizeof(hc->cop2_regs));

                if (h->wakeupsPendientes > 0) {
                    h->wakeupsPendientes--;
                }
                h->estado = HILO_LISTO;

                printf("TRADUCTOR: StartThread encolado entry=0x%08X sp=0x%08X uid=0x%08X\n",
                    (unsigned int)h->entry, (unsigned int)hc->gpr[29], (unsigned int)thid);

                g_trazaHiloNuevo = 15; // trazar las proximas 15 instrucciones SIN CONDICION,
                                       // sea del hilo que sea, para cazar exactamente cual
                                       // es la que crashea despues de esta linea.

                return 0;
            }

        case 0x82826F70: { // sceKernelSleepThreadCB() - duerme hasta WakeupThread
            Hilo *h = &g_hilos[g_hiloActual];
            if (h->wakeupsPendientes > 0) {
                h->wakeupsPendientes--; // el wakeup ya habia llegado: no se duerme
            } else {
                h->estado = HILO_DURMIENDO;
            }
            return 0; // el cambio de contexto real pasa despues, en el bucle principal
        }

        case 0xD59EAD2F: { // sceKernelWakeupThread(thid)
            u32 thid = a0;
            int idx = -1;
            for (int i = 0; i < g_cantidadHilos; i++) {
                if (g_hilos[i].uid == thid) { idx = i; break; }
            }
            if (idx == -1) return 0x80020001;

            if (g_hilos[idx].estado == HILO_DURMIENDO) {
                g_hilos[idx].estado = HILO_LISTO;
            } else {
                g_hilos[idx].wakeupsPendientes++; // wakeup adelantado, ver SleepThreadCB
            }
            return 0;
        }

        case 0x809CE29B: { // sceKernelExitDeleteThread(exitStatus) - el hilo termina y se borra solo
            g_hilos[g_hiloActual].estado = HILO_TERMINADO;
            // NO llamar planificar() aca adentro: el $v0 de este syscall
            // se escribe sobre "cpu" justo despues de este return, y si
            // ya cambiamos de hilo ese v0 le pisaria el registro a OTRO
            // hilo. El cambio de contexto real lo hace el chequeo del
            // final del while principal (mismo mecanismo que SleepThreadCB).
            return 0;
        }

        case 0x6D212BAC: { // sceKernelWaitSemaCB(semaid, count, timeout)
            u32 id = a0;
            s32 count = (s32)a1;
            if (id < MAX_SEMAFOROS) {
                if (g_semaCount[id] >= count) {
                    g_semaCount[id] -= count;
                } else {
                    g_hilos[g_hiloActual].estado = HILO_ESPERANDO_SEMA;
                    g_hilos[g_hiloActual].semaEsperado = id;
                    g_hilos[g_hiloActual].semaCantidad = count;
                }
            }
            return 0;
        }

        case 0x3F53E640: { // sceKernelSignalSema(semaid, count)
            u32 id = a0;
            s32 count = (s32)a1;
            if (id < MAX_SEMAFOROS) {
                g_semaCount[id] += count;
                hilo_revisarEsperasDeSemaforo(id);
            }
            return 0;
        }

        // ===================== MESSAGE PIPES =====================
        // Modelo: cola interna NO acotada. SendMsgPipe encola siempre (nunca
        // se bloquea el emisor) y cede CPU; ReceiveMsgPipe se bloquea en
        // HILO_ESPERANDO_PIPE hasta que haya un mensaje, y un SendMsgPipe lo
        // despierta (entrega directa o lo deja correr para que vacie la cola).
        case 0x7C0DC2A0: { // sceKernelCreateMsgPipe(name, attr, bufSize, option)
            u32 id = g_msgpipeSiguiente++;
            int slot = -1;
            for (int i = 0; i < MAX_MSGPIPES; i++) if (!g_msgpipes[i].used) { slot = i; break; }
            if (slot < 0) return 0x80020001; // sin slots libres
            MsgPipe *p = &g_msgpipes[slot];
            p->used = 1; p->id = id; p->head = p->tail = NULL; p->nWaitRecv = 0;
            printf("TRADUCTOR: CreateMsgPipe id=0x%08X\n", (unsigned int)id);
            return id;
        }

        case 0xF0B7DA1C: { // sceKernelDeleteMsgPipe(id)
            int i = msgpipe_buscarPorId(a0);
            if (i >= 0) {
                MsgNode *n = g_msgpipes[i].head;
                while (n) { MsgNode *nx = n->next; free(n->data); free(n); n = nx; }
                g_msgpipes[i].used = 0;
            }
            return 0;
        }

        case 0x876DBFAD: { // sceKernelSendMsgPipe(pid, msg, size, unblock, *pResult, priority)
            int i = msgpipe_buscarPorId(a0);
            if (i < 0) return 0x80020001;
            MsgPipe *p = &g_msgpipes[i];
            u32 size = a2;
            u32 pResultPtr = memLeer32(cpu.gpr[29] + 0x10);
            u8 *src = (u8*)resolverDireccion(a1);

            if (p->nWaitRecv > 0) {
                // Entrega directa al primer receptor bloqueado.
                int ridx = p->waitRecv[0];
                for (int k = 1; k < p->nWaitRecv; k++) p->waitRecv[k-1] = p->waitRecv[k];
                p->nWaitRecv--;
                u8 *rdst = (u8*)resolverDireccion(g_hilos_cpu[ridx].gpr[5]); // a1 del receptor = buffer
                u32 copiados = (size < g_hilos_cpu[ridx].gpr[6]) ? size : g_hilos_cpu[ridx].gpr[6]; // a2 del receptor = size esperado
                for (u32 k = 0; k < copiados; k++) rdst[k] = src[k];
                u32 rpPtr = memLeer32(g_hilos_cpu[ridx].gpr[29] + 0x10);
                u32 *rp = (u32*)resolverDireccion(rpPtr); if (rp) *rp = copiados;
                g_hilos[ridx].estado = HILO_LISTO;
                if (pResultPtr) { u32 *pr = (u32*)resolverDireccion(pResultPtr); if (pr) *pr = copiados; }
                return 0;
            }

            // Encolar en la cola interna.
            MsgNode *n = (MsgNode*)malloc(sizeof(MsgNode));
            if (!n) return 0x80020001;
            n->data = (u8*)malloc(size ? size : 1);
            n->size = size; n->next = NULL;
            for (u32 k = 0; k < size; k++) n->data[k] = src[k];
            if (p->tail) p->tail->next = n; else p->head = n;
            p->tail = n;
            if (pResultPtr) { u32 *pr = (u32*)resolverDireccion(pResultPtr); if (pr) *pr = size; }
            g_forzarCesion = 1; // darle turno al worker que lee la cola
            return 0;
        }

        case 0x884C9F90: { // sceKernelReceiveMsgPipe (BLOQUEANTE: si vacio, espera)
            int i = msgpipe_buscarPorId(a0);
            if (i < 0) return 0x80020001;
            MsgPipe *p = &g_msgpipes[i];
            u32 size = a2;
            u32 pResultPtr = memLeer32(cpu.gpr[29] + 0x10);
            u8 *dst = (u8*)resolverDireccion(a1);

            if (p->head) {
                MsgNode *n = p->head;
                p->head = n->next;
                if (!p->head) p->tail = NULL;
                u32 copiados = (n->size < size) ? n->size : size;
                for (u32 k = 0; k < copiados; k++) dst[k] = n->data[k];
                if (pResultPtr) { u32 *pr = (u32*)resolverDireccion(pResultPtr); if (pr) *pr = copiados; }
                free(n->data); free(n);
                return 0;
            }
            // Cola vacia: bloquear el hilo hasta que llegue un mensaje.
            if (a3 != 0) return 0x8002006B; // PSP_MESSAGE_PIPE_EMPTY
            g_hilos[g_hiloActual].estado = HILO_ESPERANDO_PIPE;
            g_hilos[g_hiloActual].pipeEsperado = (u32)i;
            if (g_msgpipes[i].nWaitRecv < TRADUCTOR_MAX_THREADS)
                g_msgpipes[i].waitRecv[g_msgpipes[i].nWaitRecv++] = g_hiloActual;
            return 0; // ejecutarUnPaso hace el switch de contexto
        }
        case 0xFBFA697D: { // sceKernelReceiveMsgPipeCB (NO BLOQUEANTE: variante callback)
            int i = msgpipe_buscarPorId(a0);
            if (i < 0) return 0x80020001;
            MsgPipe *p = &g_msgpipes[i];
            u32 size = a2;
            u32 pResultPtr = memLeer32(cpu.gpr[29] + 0x10);
            u8 *dst = (u8*)resolverDireccion(a1);

            if (p->head) {
                MsgNode *n = p->head;
                p->head = n->next;
                if (!p->head) p->tail = NULL;
                u32 copiados = (n->size < size) ? n->size : size;
                for (u32 k = 0; k < copiados; k++) dst[k] = n->data[k];
                if (pResultPtr) { u32 *pr = (u32*)resolverDireccion(pResultPtr); if (pr) *pr = copiados; }
                free(n->data); free(n);
                return 0;
            }
            // Cola vacia: NO bloquear (es la variante CB). Retornar error de pipe vacio.
            return 0x8002006B; // PSP_MESSAGE_PIPE_EMPTY
        }

        case 0xDF52098F: { // sceKernelPollMsgPipe(pid, msg, size, unblock, *pResult, priority)
            int i = msgpipe_buscarPorId(a0);
            if (i < 0) return 0x80020001;
            MsgPipe *p = &g_msgpipes[i];
            u32 pResultPtr = memLeer32(cpu.gpr[29] + 0x10);
            if (p->head) {
                u32 size = a2;
                u8 *dst = (u8*)resolverDireccion(a1);
                MsgNode *n = p->head;
                p->head = n->next;
                if (!p->head) p->tail = NULL;
                u32 copiados = (n->size < size) ? n->size : size;
                for (u32 k = 0; k < copiados; k++) dst[k] = n->data[k];
                if (pResultPtr) { u32 *pr = (u32*)resolverDireccion(pResultPtr); if (pr) *pr = copiados; }
                free(n->data); free(n);
                return 0;
            }
            return 0x8002006B; // vacio
        }

        case 0x1FB15A32: { // sceKernelCancelMsgPipe(pid, *pResult)
            int i = msgpipe_buscarPorId(a0);
            if (i < 0) return 0x80020001;
            MsgPipe *p = &g_msgpipes[i];
            u32 count = 0;
            MsgNode *n = p->head;
            while (n) { count++; MsgNode *nx = n->next; free(n->data); free(n); n = nx; }
            p->head = p->tail = NULL;
            // Despertar receptores bloqueados con error.
            for (int k = 0; k < p->nWaitRecv; k++) {
                int ridx = p->waitRecv[k];
                u32 rpPtr = memLeer32(g_hilos_cpu[ridx].gpr[29] + 0x10);
                u32 *rp = (u32*)resolverDireccion(rpPtr); if (rp) *rp = 0;
                g_hilos[ridx].estado = HILO_LISTO;
            }
            p->nWaitRecv = 0;
            u32 pResultPtr = memLeer32(cpu.gpr[29] + 0x10);
            if (pResultPtr) { u32 *pr = (u32*)resolverDireccion(pResultPtr); if (pr) *pr = count; }
            return 0;
        }

        case 0xD8B73127: { // sceKernelGetModuleIdByAddress(void *address)
            return 0x20000; // UID fijo del unico modulo que existe
        }

        case 0x092968F4: // sceKernelCpuSuspendIntr - devuelve estado previo (dummy)
            return 0;

        case 0x5F10D406: // sceKernelCpuResumeIntr(estado) - no-op
            return 0;

        case 0x13A5ABEF: { // sceKernelPrintf(const char *format, ...)
            char buffer[256];
            int i = 0;
            u32 direccion = a0;
            while (i < 255) {
                u8 c = memLeer8(direccion + i);
                if (c == 0) break;
                buffer[i] = (char)c;
                i++;
            }
            buffer[i] = 0;
            printf("SCE_PRINTF: %s", buffer);
            return 0;
        }

        case 0x79D1C3FA: // sceKernelDcacheWritebackAll - no-op, no simulamos cache
            return 0;

        case 0xE81CAF8F: { // sceKernelCreateCallback(name, func, arg)
            u32 uid = 0x30000 + g_cantidadCallbacks;
            if (g_cantidadCallbacks < TRADUCTOR_MAX_CALLBACKS) {
                g_callbacks[g_cantidadCallbacks].uid = uid;
                g_callbacks[g_cantidadCallbacks].func = a1;
                g_callbacks[g_cantidadCallbacks].arg = a2;
                g_cantidadCallbacks++;
                printf("TRADUCTOR: CreateCallback func=0x%08X arg=0x%08X uid=0x%08X\n",
                    (unsigned int)a1, (unsigned int)a2, (unsigned int)uid);
            } else {
                printf("TRADUCTOR: limite de callbacks alcanzado\n");
                uid = 0x80020001;
            }
            return uid;
        }

        case 0x4AC57943: // sceKernelRegisterExitCallback(cbid) - no-op, no disparamos exit real
            return 0;

        case 0xAEE7404D: // sceUmdRegisterUMDCallBack(cbid) - no-op
            return 0;

        case 0xA5DA2406: // sceUtilityLoadAvModule
            return 0; // Return SCE_OK in $v0

        case 0x36AA6E91: // sceImposeGetHomePopup
            return 1; // Return 1 (Home popup enabled) or 0

        case 0x19CFF145: // sceKernelReferThreadStatus
            // Param a0 = SceUID thid
            // Param a1 = SceKernelThreadInfo *info
            if (a1) {
                // Mock status structure if pointer non-null
                // Set info->status = PSP_THREAD_RUNNING (0x01)
                *(uint32_t*)(a1 + 0x2C) = 0x01; 
            }
            return 0; // Return SCE_OK

        case 0xC69BEBCE: // sceOpenPSIDGetOpenPSID
            if (a0) {
                // Param a0 = SceOpenPSID *psid (16-byte structure)
                memset((void*)a0, 0x01, 16); // Mock dummy 16-byte PSID
            }
            return 0; // Return SCE_OK

        case 0xC07BB470: // ThreadManForUser stub / sceKernelLsa
            return 0; // Return SCE_OK

        case 0xD979E9BF: // sceKernelUpgradeCapability
            return 0; // Return SCE_OK

        case 0xBEA46419: // sceKernelReferGlobalProfiler (Kernel_Library)
            return 0; // Return SCE_OK

        case 0x15B6446B: // sceKernelReferSysDev (Kernel_Library)
            return 0; // Return SCE_OK

        case 0xE7C27D1B: // sceRtcGetCurrentTick
            if (a0) {
                g_fakeTick += 10000;
                *(u64*)(a0) = g_fakeTick;
            }
            return 0;

        case 0x237DBD4F: { // sceKernelAllocPartitionMemory
            u32 tamano = a3 ? a3 : 0x10000;
            if (tamano > 0x200000) tamano = 0x200000;

            void *mem;
            if (g_partitionPool &&
                g_partitionPoolOffset + tamano <= g_partitionPoolSize) {
                // usar pool pre-reservada (direccion segura, fuera del BSS del modulo)
                mem = g_partitionPool + g_partitionPoolOffset;
                // alinear a 64 bytes
                g_partitionPoolOffset = (g_partitionPoolOffset + tamano + 63) & ~63;
            } else {
                mem = malloc(tamano);
                if (!mem) return 0x80000022;
            }

            u32 uid = 0x40000 + g_cantidadBloques;
            if (g_cantidadBloques < TRADUCTOR_MAX_BLOQUES_MEM) {
                g_bloquesMem[g_cantidadBloques].uid  = uid;
                g_bloquesMem[g_cantidadBloques].addr = (u32)mem;
                g_bloquesMem[g_cantidadBloques].size = tamano;
                g_cantidadBloques++;
            }
            printf("ALLOC[%u] size=0x%X addr=0x%08X uid=0x%X\n",
                (unsigned int)(g_cantidadBloques-1),
                (unsigned int)tamano, (unsigned int)mem, (unsigned int)uid);
            return uid;
        }

        case 0x9D9A5BA1: { // sceKernelGetBlockHeadAddr(uid)
            u32 uid = a0;
            for (u32 i = 0; i < g_cantidadBloques; i++) {
                if (g_bloquesMem[i].uid == uid)
                    return g_bloquesMem[i].addr;
            }
            printf("TRADUCTOR: GetBlockHeadAddr uid=0x%X no encontrado\n", (unsigned int)uid);
            return 0;
        }

        case 0xB6D61D02: // sceKernelFreePartitionMemory(uid) - no-op por ahora
            return 0;

        case 0x1F4011E6: // sceCtrlSetSamplingMode(mode) - no-op
            return 0;

        case 0x210EAB3A: // sceDisplayGetAccumulatedHcount - contador de hcount, simulado
            return g_contadorInstrucciones; // crece naturalmente, sirve como timer fake

        case 0x9C6EAAD7: // sceDisplayGetVcount - contador de vblanks
            return g_contadorInstrucciones / 286; // ~286 hlines por frame a 60fps

        case 0x0E20F177: // sceDisplaySetFrameBuf - setear framebuffer
            return 0;

        case 0x289D82FE: // sceDisplaySetFrameBuf (alias segun firmware)
            return 0;

        case 0xC6183D47: // sceUmdActivate(unit, drive) - simular disco presente
            return 0;

        case 0xE83742BA: // sceUmdDeactivate(unit, drive)
            return 0;

        case 0x46EBB729: // sceUmdCheckMedium - devolver 1 = disco presente
            return 1;

        case 0x6B4A146C: // sceUmdGetDriveStat - devolver estado "listo"
            // PSP_UMD_PRESENT | PSP_UMD_READY | PSP_UMD_READABLE = 0x12 | 0x02 | 0x20 = 0x32
            return 0x32;

        case 0x8EF08FCE: // sceUmdWaitDriveStat(stat) - ya tiene el estado, no esperar
            return 0;

        case 0x56202973: // sceUmdWaitDriveStatWithTimer(stat, timeout)
            return 0;

        case 0x4A9E5E29: // sceUmdWaitDriveStatCB(stat, timeout)
            return 0;

        case 0x6AF9B50A: // sceUmdCancelWaitDriveStat
            return 0;

        case 0x20628E6F: // sceUmdGetErrorStat
            return 0;

        case 0x05572A5F: // sceKernelExitGame() - NO salir de verdad. Se llama
                         // desde el callback estandar de salida/suspension del
                         // SDK, que estamos disparando cada tick de vblank sin
                         // que haya un evento real de "boton Home apretado" -
                         // seria un falso positivo matar la emulacion por esto.
            return 0;

        case 0xA14F40B2: // sceKernelVolatileMemTryLock(unit, addr, size) - no
                         // tenemos memoria volatil real reservada, fallar
                         // "amablemente" (el juego debe manejar esto como
                         // "no disponible" y seguir sin usarla)
            return (u32)-1;

        case 0x2A2B3DE0: // sceUtilityLoadModule(id) - fingir que cargo OK
            return 0;

        case 0xE49BFE92: // sceUtilityUnloadModule(id)
            return 0;

        case 0x54F5FB11: { // sceIoOpen(path, flags, mode)
            const char *path = (const char *)resolverDireccion(a0);
            char rutaReal[TRADUCTOR_DIR_MAXLEN + 256];
            traducirRutaPSP(path, rutaReal, sizeof(rutaReal));

            // Buscar un slot libre en la tabla de archivos abiertos
            int slot = -1;
            for (int i = 0; i < TRADUCTOR_MAX_ARCHIVOS; i++) {
                if (!g_archivosAbiertos[i].usado) { slot = i; break; }
            }
            if (slot == -1) {
                printf("TRADUCTOR: sceIoOpen(\"%s\") - sin slots libres (max %d)\n",
                    path, TRADUCTOR_MAX_ARCHIVOS);
                return -1;
            }

            // PSP_O_WRONLY=0x0002, PSP_O_RDWR=0x0003 (bits bajos de flags).
            const char *modo = "rb";
            if ((a1 & 0x3) == 0x2) modo = "wb";
            else if ((a1 & 0x3) == 0x3) modo = "r+b";

            FILE *f = fopen(rutaReal, modo);
            if (f) {
                g_archivosAbiertos[slot].usado = 1;
                g_archivosAbiertos[slot].esDir = 0;
                g_archivosAbiertos[slot].f = f;
                g_archivosAbiertos[slot].d = NULL;
                g_archivosAbiertos[slot].rutaDir[0] = '\0';
                printf("TRADUCTOR: sceIoOpen(\"%s\") -> \"%s\" OK fd=%d\n", path, rutaReal, slot + 3);
                return slot + 3;
            }
            // Si no es un archivo comun, probamos como directorio. kjfs abre
            // "fatms0:" (la raiz del memory stick), que es un directorio, y
            // luego lo enumera con sceIoDread sobre este mismo fd.
            DIR *dir = opendir(rutaReal);
            if (dir) {
                g_archivosAbiertos[slot].usado = 1;
                g_archivosAbiertos[slot].esDir = 1;
                g_archivosAbiertos[slot].f = NULL;
                g_archivosAbiertos[slot].d = dir;
                strncpy(g_archivosAbiertos[slot].rutaDir, rutaReal, sizeof(g_archivosAbiertos[slot].rutaDir) - 1);
                g_archivosAbiertos[slot].rutaDir[sizeof(g_archivosAbiertos[slot].rutaDir) - 1] = '\0';
                printf("TRADUCTOR: sceIoOpen(\"%s\") -> \"%s\" (DIR) OK fd=%d\n", path, rutaReal, slot + 3);
                return slot + 3;
            }
            printf("TRADUCTOR: sceIoOpen(\"%s\") -> \"%s\" NO ENCONTRADO\n", path, rutaReal);
            return -1;
        }

        case 0x6A638D83: { // sceIoRead(fd, buf, size)
            int slot = (int)a0 - 3;
            if (slot < 0 || slot >= TRADUCTOR_MAX_ARCHIVOS || !g_archivosAbiertos[slot].usado || g_archivosAbiertos[slot].esDir) return -1;
            void *buf = resolverDireccion(a1);
            size_t leidos = fread(buf, 1, a2, g_archivosAbiertos[slot].f);
            return (u32)leidos;
        }

        case 0x810C4BC3: { // sceIoClose(fd)
            int slot = (int)a0 - 3;
            if (slot < 0 || slot >= TRADUCTOR_MAX_ARCHIVOS || !g_archivosAbiertos[slot].usado) return -1;
            if (g_archivosAbiertos[slot].esDir) { if (g_archivosAbiertos[slot].d) closedir(g_archivosAbiertos[slot].d); }
            else { if (g_archivosAbiertos[slot].f) fclose(g_archivosAbiertos[slot].f); }
            g_archivosAbiertos[slot].usado = 0;
            g_archivosAbiertos[slot].f = NULL;
            g_archivosAbiertos[slot].d = NULL;
            return 0;
        }

        case 0xFF5940B6: { // sceIoLseek(fd, offset, whence) - offset de 64 bits (a2:a3)
            int slot = (int)a0 - 3;
            if (slot < 0 || slot >= TRADUCTOR_MAX_ARCHIVOS || !g_archivosAbiertos[slot].usado || g_archivosAbiertos[slot].esDir) return (u32)-1;
            long offset = (long)a1;
            int origen = (a2 == 0) ? SEEK_SET : (a2 == 1) ? SEEK_CUR : SEEK_END;
            fseek(g_archivosAbiertos[slot].f, offset, origen);
            return (u32)ftell(g_archivosAbiertos[slot].f);
        }

        case 0x27EB27B8: { // sceIoLseek32(fd, offset, whence)
            int slot = (int)a0 - 3;
            if (slot < 0 || slot >= TRADUCTOR_MAX_ARCHIVOS || !g_archivosAbiertos[slot].usado || g_archivosAbiertos[slot].esDir) return (u32)-1;
            int origen = (a2 == 0) ? SEEK_SET : (a2 == 1) ? SEEK_CUR : SEEK_END;
            fseek(g_archivosAbiertos[slot].f, (long)(s32)a1, origen);
            return (u32)ftell(g_archivosAbiertos[slot].f);
        }

        case 0xACE946E8: { // sceIoGetstat(path, SceIoStat *stat)
            const char *path = (const char *)resolverDireccion(a0);
            char rutaReal[TRADUCTOR_DIR_MAXLEN + 256];
            traducirRutaPSP(path, rutaReal, sizeof(rutaReal));
            struct stat st;
            if (stat(rutaReal, &st) != 0) return -1;
            u8 *sb = (u8*)resolverDireccion(a1);
            if (!sb) return -1;
            // SceIoStat: mode(0) u32, attr(4) u32, size u64(8 lo / 12 hi),
            // ctime(16) atime(24) mtime(32) [8 bytes c/u], hisize(40) u32
            int esDir = S_ISDIR(st.st_mode);
            memEscribir32((u32)(sb + 0),  esDir ? 0x4000 : 0x2000); // SCE_STM_FDIR / SCE_STM_FREG
            memEscribir32((u32)(sb + 4),  esDir ? 0x10 : 0x20);     // atributo de dir/archivo
            u64 sz = (u64)st.st_size;
            memEscribir32((u32)(sb + 8),  (u32)(sz & 0xFFFFFFFF));
            memEscribir32((u32)(sb + 12), (u32)(sz >> 32));
            memEscribir32((u32)(sb + 40), (u32)(sz >> 32));          // hisize (redundante)
            return 0;
        }

        case 0xA0B5A7C2: { // sceIoDopen(path) -> fd de directorio
            const char *path = (const char *)resolverDireccion(a0);
            char rutaReal[TRADUCTOR_DIR_MAXLEN + 256];
            traducirRutaPSP(path, rutaReal, sizeof(rutaReal));
            int slot = -1;
            for (int i = 0; i < TRADUCTOR_MAX_ARCHIVOS; i++) {
                if (!g_archivosAbiertos[i].usado) { slot = i; break; }
            }
            if (slot == -1) return -1;
            DIR *dir = opendir(rutaReal);
            if (!dir) return -1;
            g_archivosAbiertos[slot].usado = 1;
            g_archivosAbiertos[slot].esDir = 1;
            g_archivosAbiertos[slot].f = NULL;
            g_archivosAbiertos[slot].d = dir;
            strncpy(g_archivosAbiertos[slot].rutaDir, rutaReal, sizeof(g_archivosAbiertos[slot].rutaDir) - 1);
            g_archivosAbiertos[slot].rutaDir[sizeof(g_archivosAbiertos[slot].rutaDir) - 1] = '\0';
            return slot + 3;
        }

        case 0xE23EEC33: { // sceIoDread(fd, SceIoDirent *buf)
            int slot = (int)a0 - 3;
            if (slot < 0 || slot >= TRADUCTOR_MAX_ARCHIVOS || !g_archivosAbiertos[slot].usado ||
                !g_archivosAbiertos[slot].esDir || !g_archivosAbiertos[slot].d) return -1;
            struct dirent *de = readdir(g_archivosAbiertos[slot].d);
            if (!de) return 0; // fin de la enumeracion
            u8 *buf = (u8*)resolverDireccion(a1);
            if (buf) {
                // SceIoDirent: d_stat(64 bytes @0x00), d_name[256] @0x40, d_private @0x140, dummy @0x144
                for (int k = 0; k < 16; k++) memEscribir32((u32)(buf + k*4), 0); // limpiar d_stat
                char entrada[TRADUCTOR_DIR_MAXLEN + 256 + 16];
                snprintf(entrada, sizeof(entrada), "%s/%s",
                         g_archivosAbiertos[slot].rutaDir, de->d_name);
                struct stat st;
                int esDir = 0;
                if (stat(entrada, &st) == 0) esDir = S_ISDIR(st.st_mode);
                else if (de->d_name[0] == '.') esDir = 1;
                memEscribir32((u32)(buf + 0),  esDir ? 0x4000 : 0x2000);
                memEscribir32((u32)(buf + 4),  esDir ? 0x10 : 0x20);
                u64 sz = (stat(entrada, &st) == 0) ? (u64)st.st_size : 0;
                memEscribir32((u32)(buf + 8),  (u32)(sz & 0xFFFFFFFF));
                memEscribir32((u32)(buf + 12), (u32)(sz >> 32));
                for (int k = 0; k < 255; k++) {
                    u8 c = (k < (int)strlen(de->d_name)) ? (u8)de->d_name[k] : 0;
                    memEscribir8((u32)(buf + 0x40 + k), c);
                }
                memEscribir8((u32)(buf + 0x40 + 255), 0);
            }
            return 1; // una entrada leida
        }

        // kjfs - libreria propietaria de Konami, 51 funciones sin documentar
        // Todas retornan 0 por defecto con log para poder identificarlas por contexto
        case 0x0120E8BE: case 0x091CC7DB: case 0x0B4EDE89: case 0x0C7A5E8C:
        case 0x0EA79943: case 0x18B0F227: case 0x1B969771: case 0x1ECF095D:
        case 0x21582D1E: case 0x2725DF14: case 0x2B71773D: case 0x2B9B31B4:
        case 0x30557549: case 0x3245F846: case 0x432F631B: case 0x447CC6C0:
        case 0x482E3F1A: case 0x4CB97AD1: case 0x52C298C5: case 0x61558B95:
        case 0x73DE0032: case 0x78A29747: case 0x813E06AD: case 0x833DE7FC:
        case 0x87AE0285: case 0x8AF47001: case 0x8E71A56F: case 0x8F8DF1BB:
        case 0x93E9297F: case 0x9995DE66: case 0x9AAA1137: case 0x9BB01FF2:
        case 0xA485E39E: case 0xA9E66218: case 0xAF65B85A: case 0xB2756FFA:
        case 0xB60FBBB0: case 0xB6590F47: case 0xB82B0E1A: case 0xBA628533:
        case 0xBFF7360B: case 0xC51F367F: case 0xCC214583: case 0xCF0188CF:
        case 0xD235A288: case 0xD4AD7362: case 0xD5E3EFF1: case 0xDC49F429:
        case 0xDE47B76B: case 0xF6A3F12F: case 0xFDC25446:
            printf("kjfs 0x%08X a0=0x%08X a1=0x%08X a2=0x%08X a3=0x%08X\n",
                (unsigned int)nid, a0, a1, a2, a3);
            return 0;

        case 0xE47E40E4: // sceGeEdramGetAddr() - base del VRAM
            // Puntero real dentro del propio ELF (32 bits nativo, no hay
            // truncamiento posible aca - esto no es un proceso de host).
            return (u32)g_vramBuffer;

        case 0x1F6752AD: // sceGeEdramGetSize() - tamano del VRAM (2MB en PSP)
            return 0x00200000;

        case 0xB77905EA: // sceGeEdramSetAddrTranslation(size) - no-op
            return 0;

        case 0xAB49E76A: { // sceGeListEnQueue(list, stall, cbid, arg) - encola display list
            u32 direccionListaReal = a0 & 0x1FFFFFFF;
            static int g_listasProcesadas = 0;
            int mostrarDetalle = (g_listasProcesadas < 5);
            g_listasProcesadas++;

            if (mostrarDetalle) {
                printf("sceGeListEnQueue #%d list=0x%08X (real=0x%08X) stall=0x%08X\n",
                    g_listasProcesadas, a0, direccionListaReal, a1);
            }
            geProcesarLista(direccionListaReal, mostrarDetalle);
            if (mostrarDetalle) {
                geImprimirHistograma();
            } else if (g_listasProcesadas == 6) {
                printf("TRADUCTOR GE: ya se mostraron 5 listas en detalle, "
                       "se sigue procesando el resto en silencio.\n");
            }
            return 0x1234; // UID fake de la lista
        }

        case 0xB287BD61: { // sceGeDrawSync(mode) - esperar que termine el GE
            static int g_vecesDrawSync = 0;
            if (g_vecesDrawSync < 5) {
                printf("sceGeDrawSync #%d mode=0x%08X\n", g_vecesDrawSync + 1, a0);
                g_vecesDrawSync++;
            }
            return 0;
        }

        case 0x03444EB4: // sceGeListSync(qid, mode)
            return 0;

        case 0xA4FC06A4: // sceGeSetCallback
            return 0x5678;

        case 0x05DB22CE: // sceGeUnsetCallback
            return 0;

        case 0xDC93CFEF: // sceGeGetCmd
            return 0;

        case 0x4C06E472: // sceGeContinue
            return 0;

        case 0xB448EC0D: // sceGeBreak
            return 0;

        case 0x1C0D95A6: { // sceGeListEnQueueHead(list, stall, cbid, arg)
            static int g_vecesEnQueueHead = 0;
            u32 direccionListaReal = a0 & 0x1FFFFFFF;
            int mostrarDetalle = (g_vecesEnQueueHead < 5);
            g_vecesEnQueueHead++;
            if (mostrarDetalle) {
                printf("sceGeListEnQueueHead #%d list=0x%08X (real=0x%08X)\n",
                    g_vecesEnQueueHead, a0, direccionListaReal);
            }
            geProcesarLista(direccionListaReal, mostrarDetalle);
            if (mostrarDetalle) geImprimirHistograma();
            return 0x1235;
        }

        case 0xE0D68148: // sceGeListUpdateStallAddr(id, stallAddr) - actualiza stall de lista ya encolada
            return 0;

        case 0xCA04A2B9: { // sceKernelRegisterSubIntrHandler(subIntr, handler, arg)
            printf("sceKernelRegisterSubIntrHandler subIntr=0x%08X handler=0x%08X\n", a0, a1);
            if (a1 != 0 && g_cantidadSubIntr < TRADUCTOR_MAX_SUBINTR) {
                g_subIntr[g_cantidadSubIntr].subIntr = a0;
                g_subIntr[g_cantidadSubIntr].handler = a1;
                g_subIntr[g_cantidadSubIntr].common = a2;
                g_subIntr[g_cantidadSubIntr].habilitado = 0; // se habilita con EnableSubIntr
                g_cantidadSubIntr++;
            }
            return 0;
        }

        case 0xFB8E22EC: { // sceKernelEnableSubIntr(subIntr, arg)
            for (int i = 0; i < g_cantidadSubIntr; i++) {
                if (g_subIntr[i].subIntr == a0) g_subIntr[i].habilitado = 1;
            }
            return 0;
        }

        case 0xD6DA4BA1: { // sceKernelCreateSema(name, attr, initVal, maxVal, option)
            s32 id = g_semaSiguiente++;
            if ((u32)id < MAX_SEMAFOROS) g_semaCount[id] = (s32)a2; // a2 = initVal
            return id;
        }

        case 0x04B7766E: // scePowerSetClockFrequency(cpu, ram, bus)
            // Devuelve 0 (SCE_OK) indicando que la frecuencia de reloj se configuró correctamente
        case 0x44280A3D: // scePowerGetCpuClockFrequency
        case 0x3BDB7793: // scePowerGetBusClockFrequency
            return 0;

        case 0x4E3A1105: { // sceKernelWaitSema(semaid, count, timeout) - CORREGIDO:
                           // esto NO es sceKernelCancelAlarm (estaba mal mapeado).
                           // Confirmado contra jpcsp/pspsdk/ps2dev: 0x4E3A1105 es
                           // sceKernelWaitSema. Estaba tratado como no-op y nunca
                           // bloqueaba de verdad - eso causaba el loop infinito
                           // (dos hilos haciendo handshake sobre un semaforo que
                           // en la practica no hacia nada).
            u32 id = a0;
            s32 count = (s32)a1;
            if (id < MAX_SEMAFOROS) {
                if (g_semaCount[id] >= count) {
                    g_semaCount[id] -= count;
                } else {
                    g_hilos[g_hiloActual].estado = HILO_ESPERANDO_SEMA;
                    g_hilos[g_hiloActual].semaEsperado = id;
                    g_hilos[g_hiloActual].semaCantidad = count;
                }
            }
            return 0;
        }

        case 0x68DA9E36: // sceKernelDelayThread(SceUInt delay)
            return 0; // Return SCE_OK

        default:
            *implementado = 0;
            return 0;
    }
}

// ============================================================
// Ejecutar UNA instruccion "normal" (no de salto).
// Se usa tanto para el flujo principal como para la instruccion
// del delay slot de un salto.
// ============================================================
static void ejecutarNormal(u32 instr, u32 pcInstr) {
    g_pcEjecutando = pcInstr;
    u32 opcode = OPCODE(instr);

    if (instr == 0) {
        return; // NOP puro (sll $zero,$zero,0) - no hacer nada
    }

    switch (opcode) {

        case 0x00: { // tipo R
            u32 funct = FUNCT(instr);
            switch (funct) {
                
                case 0x21: // ADDU  rd = rs + rt
                    escribirGpr(RD(instr), cpu.gpr[RS(instr)] + cpu.gpr[RT(instr)]);
                    break;
                case 0x23: // SUBU  rd = rs - rt
                    escribirGpr(RD(instr), cpu.gpr[RS(instr)] - cpu.gpr[RT(instr)]);
                    break;
                case 0x24: // AND   rd = rs & rt
                    escribirGpr(RD(instr), cpu.gpr[RS(instr)] & cpu.gpr[RT(instr)]);
                    break;
                case 0x25: // OR    rd = rs | rt
                    escribirGpr(RD(instr), cpu.gpr[RS(instr)] | cpu.gpr[RT(instr)]);
                    break;
                case 0x26: // XOR   rd = rs ^ rt
                    escribirGpr(RD(instr), cpu.gpr[RS(instr)] ^ cpu.gpr[RT(instr)]);
                    break;
                case 0x27: // NOR   rd = ~(rs | rt)
                    escribirGpr(RD(instr), ~(cpu.gpr[RS(instr)] | cpu.gpr[RT(instr)]));
                    break;
                case 0x2B: // SLTU  rd = (rs < rt) ? 1 : 0   (sin signo)
                    escribirGpr(RD(instr), (cpu.gpr[RS(instr)] < cpu.gpr[RT(instr)]) ? 1 : 0);
                    break;
                case 0x2A: // SLT   rd = (rs < rt) ? 1 : 0   (con signo)
                    escribirGpr(RD(instr), ((s32)cpu.gpr[RS(instr)] < (s32)cpu.gpr[RT(instr)]) ? 1 : 0);
                    break;
                case 0x00: // SLL   rd = rt << shamt  (si todo es 0, es un NOP)
                    escribirGpr(RD(instr), cpu.gpr[RT(instr)] << SHAMT(instr));
                    break;
                case 0x02: // SRL   rd = rt >> shamt  (logico, sin signo)
                    escribirGpr(RD(instr), cpu.gpr[RT(instr)] >> SHAMT(instr));
                    break;
                case 0x03: // SRA   rd = rt >> shamt  (aritmetico, con signo)
                    escribirGpr(RD(instr), (u32)((s32)cpu.gpr[RT(instr)] >> SHAMT(instr)));
                    break;
                case 0x04: // SLLV  rd = rt << (rs & 0x1F)
                    escribirGpr(RD(instr), cpu.gpr[RT(instr)] << (cpu.gpr[RS(instr)] & 0x1F));
                    break;
                case 0x06: // SRLV  rd = rt >> (rs & 0x1F)  (logico, sin signo)
                    escribirGpr(RD(instr), cpu.gpr[RT(instr)] >> (cpu.gpr[RS(instr)] & 0x1F));
                    break;
                case 0x07: // SRAV  rd = rt >> (rs & 0x1F)  (aritmetico, con signo)
                    escribirGpr(RD(instr), (u32)((s32)cpu.gpr[RT(instr)] >> (cpu.gpr[RS(instr)] & 0x1F)));
                    break;

                case 0x10: // MFHI  rd = HI
                    escribirGpr(RD(instr), cpu.hi);
                    break;
                case 0x11: // MTHI  HI = rs
                    cpu.hi = cpu.gpr[RS(instr)];
                    break;
                case 0x13: // MTLO  LO = rs
                    cpu.lo = cpu.gpr[RS(instr)];
                    break;

                case 0x19: { // MULTU  HI, LO = rs * rt (sin signo)
                    u64 res = (u64)cpu.gpr[RS(instr)] * (u64)cpu.gpr[RT(instr)];
                    cpu.lo = (u32)(res & 0xFFFFFFFF);
                    cpu.hi = (u32)((res >> 32) & 0xFFFFFFFF);
                    break;
                }
                case 0x1A: { // DIV   LO = rs / rt (con signo), HI = rs % rt
                    s32 rs = (s32)cpu.gpr[RS(instr)];
                    s32 rt = (s32)cpu.gpr[RT(instr)];
                    if (rt != 0) {
                        cpu.lo = (u32)(rs / rt);
                        cpu.hi = (u32)(rs % rt);
                    }
                    break;
                }
                case 0x1B: { // DIVU  LO = rs / rt (sin signo), HI = rs % rt
                    u32 rs = cpu.gpr[RS(instr)];
                    u32 rt = cpu.gpr[RT(instr)];
                    if (rt != 0) {
                        cpu.lo = rs / rt;
                        cpu.hi = rs % rt;
                    }
                    break;
                }

                case 0x2C: // MAX  rd = (rs > rt) ? rs : rt  (con signo)
                    escribirGpr(RD(instr), ((s32)cpu.gpr[RS(instr)] > (s32)cpu.gpr[RT(instr)])
                                        ? cpu.gpr[RS(instr)] : cpu.gpr[RT(instr)]);
                    break;

                case 0x2D: // MIN  rd = (rs < rt) ? rs : rt  (con signo)
                    escribirGpr(RD(instr), ((s32)cpu.gpr[RS(instr)] < (s32)cpu.gpr[RT(instr)])
                                        ? cpu.gpr[RS(instr)] : cpu.gpr[RT(instr)]);
                    break;

                case 0x0B: // MOVN  if (rt != 0) rd = rs
                    if (cpu.gpr[RT(instr)] != 0) {
                        escribirGpr(RD(instr), cpu.gpr[RS(instr)]);
                    }
                    break;
                case 0x0A: // MOVZ  if (rt == 0) rd = rs
                    if (cpu.gpr[RT(instr)] == 0) {
                        escribirGpr(RD(instr), cpu.gpr[RS(instr)]);
                    }
                    break;

                case 0x0C: { // SYSCALL
                    u32 indiceImport = (instr >> 6) & 0xFFFFF;

                    int implementado = 0;
                    u32 resultado = resolverImport(
                        indiceImport,
                        cpu.gpr[4], cpu.gpr[5], cpu.gpr[6], cpu.gpr[7], // a0-a3
                        &implementado
                    );

                    if (!implementado) {
                        u32 nid = (indiceImport < g_cantidadImportsRegistrados) ? g_importNid[indiceImport] : 0;
                        const char *lib = (indiceImport < g_cantidadImportsRegistrados) ? g_importNombreLib[indiceImport] : "?";
                        printf(
                            "TRADUCTOR: IMPORT NO IMPLEMENTADO indice=%u nid=0x%08X libreria=%s vaddr=0x%08X%s\n",
                            (unsigned int)indiceImport, (unsigned int)nid, lib,
                            (unsigned int)(pcInstr - (u32)g_delta),
                            g_enInvocacionCallback ? " (dentro de un callback - solo se aborta el callback)" : ""
                        );
                        if (g_enInvocacionCallback) {
                            // No matar todo el interprete por esto: aborta
                            // limpio SOLO la invocacion del callback actual,
                            // el resto del juego sigue corriendo normal.
                            cpu.pc = PC_CALLBACK_SALIDA;
                            g_pcForzadoPorImport = 1; // que no se pise este pc con pcActual+4
                        } else {
                            cpu.detenido = 1;
                        }
                        break;
                    }

                    // En un hijack de modulo no escribimos $v0: la funcion del
                    // modulo lo setea ella misma al retornar.
                    if (!g_pcForzadoPorImport) escribirGpr(2, resultado); // $v0
                    break;
                }

                case 0x12: // MFLO  rd = LO
                    escribirGpr(RD(instr), cpu.lo);
                    break;

                case 0x18: { // MULT  HI, LO = rs * rt (con signo)
                    s64 res = (s64)(s32)cpu.gpr[RS(instr)] * (s64)(s32)cpu.gpr[RT(instr)];
                    cpu.lo = (u32)(res & 0xFFFFFFFF);
                    cpu.hi = (u32)((res >> 32) & 0xFFFFFFFF);
                    break;
                }

                case 0x1C: { // MADD  HI:LO = HI:LO + (rs * rt)  (con signo, Allegrex)
                    s64 producto = (s64)(s32)cpu.gpr[RS(instr)] * (s64)(s32)cpu.gpr[RT(instr)];
                    s64 acc = ((s64)(s32)cpu.hi << 32) | (u64)cpu.lo;
                    acc += producto;
                    cpu.lo = (u32)(acc & 0xFFFFFFFF);
                    cpu.hi = (u32)((acc >> 32) & 0xFFFFFFFF);
                    break;
                }
                case 0x1D: { // MADDU  HI:LO = HI:LO + (rs * rt)  (sin signo, Allegrex)
                    u64 producto = (u64)cpu.gpr[RS(instr)] * (u64)cpu.gpr[RT(instr)];
                    u64 acc = ((u64)cpu.hi << 32) | (u64)cpu.lo;
                    acc += producto;
                    cpu.lo = (u32)(acc & 0xFFFFFFFF);
                    cpu.hi = (u32)((acc >> 32) & 0xFFFFFFFF);
                    break;
                }
                case 0x1E: { // MSUB  HI:LO = HI:LO - (rs * rt)  (con signo, Allegrex)
                    s64 producto = (s64)(s32)cpu.gpr[RS(instr)] * (s64)(s32)cpu.gpr[RT(instr)];
                    s64 acc = ((s64)(s32)cpu.hi << 32) | (u64)cpu.lo;
                    acc -= producto;
                    cpu.lo = (u32)(acc & 0xFFFFFFFF);
                    cpu.hi = (u32)((acc >> 32) & 0xFFFFFFFF);
                    break;
                }
                case 0x1F: { // MSUBU  HI:LO = HI:LO - (rs * rt)  (sin signo, Allegrex)
                    u64 producto = (u64)cpu.gpr[RS(instr)] * (u64)cpu.gpr[RT(instr)];
                    u64 acc = ((u64)cpu.hi << 32) | (u64)cpu.lo;
                    acc -= producto;
                    cpu.lo = (u32)(acc & 0xFFFFFFFF);
                    cpu.hi = (u32)((acc >> 32) & 0xFFFFFFFF);
                    break;
                }
                case 0x30: { // ROTR  rd = rt rotado a la derecha por shamt bits (Allegrex / MIPS32R2)
                    u32 rt = cpu.gpr[RT(instr)];
                    u32 sa = SHAMT(instr);
                    escribirGpr(RD(instr), (rt >> sa) | (rt << (32 - sa)));
                    break;
                }
                case 0x32: { // ROTRV rd = rt rotado a la derecha por rs bits (Allegrex / MIPS32R2)
                    u32 rt = cpu.gpr[RT(instr)];
                    u32 sa = cpu.gpr[RS(instr)] & 0x1F;
                    escribirGpr(RD(instr), (rt >> sa) | (rt << (32 - sa)));
                    break;
                }
                case 0x20: { // ADD (con signo, trap en overflow)
                    s32 a = (s32)cpu.gpr[RS(instr)];
                    s32 b = (s32)cpu.gpr[RT(instr)];
                    escribirGpr(RD(instr), (u32)(a + b));
                    break;
                }
                case 0x22: { // SUB (con signo, trap en overflow)
                    s32 a = (s32)cpu.gpr[RS(instr)];
                    s32 b = (s32)cpu.gpr[RT(instr)];
                    escribirGpr(RD(instr), (u32)(a - b));
                    break;
                }

                default:
                    logNoImplementado(instr, pcInstr, "tipo-R funct desconocido");
                    break;
            }
            break;
        }

        case 0x0A: // SLTI   rt = (rs < inmediato_con_signo) ? 1 : 0   (con signo)
            escribirGpr(RT(instr), ((s32)cpu.gpr[RS(instr)] < IMM16_SEXT(instr)) ? 1 : 0);
            break;

        case 0x0B: // SLTIU  rt = (rs < inmediato) ? 1 : 0   (sin signo, imm extendido con signo)
            escribirGpr(RT(instr), (cpu.gpr[RS(instr)] < (u32)IMM16_SEXT(instr)) ? 1 : 0);
            break;

        case 0x08: // ADDIU  rt = rs + inmediato_con_signo
            escribirGpr(RT(instr), cpu.gpr[RS(instr)] + IMM16_SEXT(instr));
            break;

        case 0x09: // ADDIU (idem, algunos ensambladores usan este opcode indistintamente)
            escribirGpr(RT(instr), cpu.gpr[RS(instr)] + IMM16_SEXT(instr));
            break;

        case 0x0C: // ANDI   rt = rs & inmediato_SIN_signo
            escribirGpr(RT(instr), cpu.gpr[RS(instr)] & IMM16(instr));
            break;

        case 0x0D: // ORI    rt = rs | inmediato_SIN_signo
            escribirGpr(RT(instr), cpu.gpr[RS(instr)] | IMM16(instr));
            break;

        case 0x0E: // XORI   rt = rs ^ inmediato_SIN_signo
            escribirGpr(RT(instr), cpu.gpr[RS(instr)] ^ IMM16(instr));
            break;

        case 0x0F: // LUI    rt = inmediato << 16
            escribirGpr(RT(instr), IMM16(instr) << 16);
            break;

        case 0x1C: { // SPECIAL2 (MIPS32R2 / Allegrex)
            u32 funct2 = FUNCT(instr);
            switch (funct2) {
                case 0x02: // MUL   rd = (rs * rt) bajos 32 bits  (no toca HI/LO)
                    escribirGpr(RD(instr), cpu.gpr[RS(instr)] * cpu.gpr[RT(instr)]);
                    break;
                case 0x30: { // CLZ   rd = cantidad de ceros a la izquierda de rs
                    u32 x = cpu.gpr[RS(instr)];
                    u32 c = 0;
                    for (int b = 31; b >= 0; b--) {
                        if ((x >> b) & 1) break;
                        c++;
                    }
                    escribirGpr(RD(instr), c);
                    break;
                }
                case 0x31: { // CLO   rd = cantidad de unos a la izquierda de rs
                    u32 x = cpu.gpr[RS(instr)];
                    u32 c = 0;
                    for (int b = 31; b >= 0; b--) {
                        if (((x >> b) & 1) == 0) break;
                        c++;
                    }
                    escribirGpr(RD(instr), c);
                    break;
                }
                default:
                    logNoImplementado(instr, pcInstr, "SPECIAL2 funct desconocido");
                    break;
            }
            break;
        }

        case 0x11: { // COP1 (FPU)
            u32 fmt = RS(instr); // reusa el campo, pero aca es un sub-opcode, no un registro

            switch (fmt) {
                case 0x00: { // MFC1  rt(GPR) = fpr[rd] (bits crudos)
                    u32 bits;
                    memcpy(&bits, &g_fpr[RD(instr)], 4);
                    escribirGpr(RT(instr), bits);
                    break;
                }
                case 0x02: { // CFC1  rt(GPR) = fcr31 (solo el registro 31 existe realmente)
                    escribirGpr(RT(instr), g_fcr31);
                    break;
                }
                case 0x04: { // MTC1  fpr[rd] = rt(GPR) (bits crudos)
                    u32 bits = cpu.gpr[RT(instr)];
                    memcpy(&g_fpr[RD(instr)], &bits, 4);
                    break;
                }
                case 0x06: { // CTC1  fcr31 = rt(GPR)
                    g_fcr31 = cpu.gpr[RT(instr)];
                    break;
                }
                case 0x10: { // fmt = S (aritmetica de simple precision)
                    u32 fd = SHAMT(instr);
                    u32 fs = RD(instr);
                    u32 ft = RT(instr);
                    u32 funct = FUNCT(instr);

                    switch (funct) {
                        case 0x00: g_fpr[fd] = g_fpr[fs] + g_fpr[ft]; break; // ADD.S
                        case 0x01: g_fpr[fd] = g_fpr[fs] - g_fpr[ft]; break; // SUB.S
                        case 0x02: g_fpr[fd] = g_fpr[fs] * g_fpr[ft]; break; // MUL.S
                        case 0x03: g_fpr[fd] = g_fpr[fs] / g_fpr[ft]; break; // DIV.S
                        case 0x04: g_fpr[fd] = sqrtf(g_fpr[fs]); break;      // SQRT.S
                        case 0x05: g_fpr[fd] = fabsf(g_fpr[fs]); break;      // ABS.S
                        case 0x06: g_fpr[fd] = g_fpr[fs]; break;             // MOV.S
                        case 0x07: g_fpr[fd] = -g_fpr[fs]; break;            // NEG.S
                        case 0x24: { // CVT.W.S  float -> entero (bits crudos van a fpr[fd])
                            s32 v = (s32)g_fpr[fs]; // trunca hacia cero; si hace falta el modo
                                                     // de redondeo real, mirar g_fcr31
                            memcpy(&g_fpr[fd], &v, 4);
                            break;
                        }
                        default:
                            logNoImplementado(instr, pcInstr, "COP1 fmt=S funct desconocido");
                            break;
                    }
                    break;
                }
                case 0x14: { // fmt = W (conversion desde entero)
                    u32 fd = SHAMT(instr);
                    u32 fs = RD(instr);
                    u32 funct = FUNCT(instr);

                    switch (funct) {
                        case 0x20: { // CVT.S.W  entero crudo en fpr[fs] -> float en fpr[fd]
                            s32 ival;
                            memcpy(&ival, &g_fpr[fs], 4);
                            g_fpr[fd] = (float)ival;
                            break;
                        }
                        default:
                            logNoImplementado(instr, pcInstr, "COP1 fmt=W funct desconocido");
                            break;
                    }
                    break;
                }
                default:
                    logNoImplementado(instr, pcInstr, "COP1 fmt desconocido (aritmetica FPU todavia no implementada)");
                    break;
            }
            break;
        }

        case 0x12: { // COP2 (VFPU / Coprocesador 2)
            u32 rs = RS(instr);
            u32 rt = RT(instr);
            u32 rd = RD(instr);

            switch (rs) {
                case 0x00: // MFC2
                    escribirGpr(rt, cpu.cop2_regs[rd]);
                    break;

                case 0x02: // CFC2
                    escribirGpr(rt, cpu.cop2_regs[rd]);
                    break;

                case 0x03: // MFV
                    escribirGpr(rt, cpu.cop2_regs[rd]);
                    break;

                case 0x04: // MTC2
                    cpu.cop2_regs[rd] = cpu.gpr[rt];
                    break;

                case 0x06: // CTC2
                    cpu.cop2_regs[rd] = cpu.gpr[rt];
                    break;

                case 0x07: // MTV
                    cpu.cop2_regs[rd] = cpu.gpr[rt];
                    break;

                default:
                    logNoImplementado(instr, pcInstr, "COP2 / VFPU no implementada");
                    break;
            }
            break;
        }

        case 0x1F: { // SPECIAL3 (ALLEGREX / MIPS32R2)
            u32 funct = FUNCT(instr);
            switch (funct) {
                case 0x00: { // EXT  rt, rs, pos, size
                    u32 pos = SHAMT(instr);
                    u32 size = RD(instr) + 1;
                    u32 mask = (size == 32) ? 0xFFFFFFFF : ((1U << size) - 1);
                    u32 val = (cpu.gpr[RS(instr)] >> pos) & mask;
                    escribirGpr(RT(instr), val);
                    break;
                }
                case 0x04: { // INS rt, rs, pos, size
                    u32 pos = SHAMT(instr);
                    u32 msb = RD(instr);
                    if (pos <= msb) {
                        u32 size = msb - pos + 1;
                        u32 mask = (size == 32) ? 0xFFFFFFFF : ((1U << size) - 1);
                        u32 src = cpu.gpr[RS(instr)] & mask;
                        u32 dest = cpu.gpr[RT(instr)] & ~(mask << pos);
                        escribirGpr(RT(instr), dest | (src << pos));
                    }
                    break;
                }
                case 0x20: { // BSHFL (SEB / SEBH)
                    u32 sa = SHAMT(instr);
                    if (sa == 0x10) { // SEBH: Sign-Extend Halfword
                        s16 val = (s16)(cpu.gpr[RT(instr)] & 0xFFFF);
                        escribirGpr(RD(instr), (u32)(s32)val);
                    } else if (sa == 0x18) { // SEB: Sign-Extend Byte
                        s8 val = (s8)(cpu.gpr[RT(instr)] & 0xFF);
                        escribirGpr(RD(instr), (u32)(s32)val);
                    }
                    break;
                }
                default:
                    logNoImplementado(instr, pcInstr, "SPECIAL3 funct desconocido");
                    break;
            }
            break;
        }

        case 0x22: { // LWL - Load Word Left
            u32 addr = cpu.gpr[RS(instr)] + IMM16_SEXT(instr);
            u32 alin = addr & 3;
            u32 base = memLeer32(addr & ~3);
            u32 reg  = cpu.gpr[RT(instr)];
            static const u32 maskL[4] = {0xFFFFFF00, 0xFFFF0000, 0xFF000000, 0x00000000};
            static const int shiftL[4] = {8, 16, 24, 0};
            u32 resultado = (base << shiftL[alin]) | (reg & maskL[alin]);
            if (alin == 3) resultado = base << 24;  // caso especial
            escribirGpr(RT(instr), resultado);
            break;
        }

        case 0x26: { // LWR - Load Word Right
            u32 addr = cpu.gpr[RS(instr)] + IMM16_SEXT(instr);
            u32 alin = addr & 3;
            u32 base = memLeer32(addr & ~3);
            u32 reg  = cpu.gpr[RT(instr)];
            static const u32 maskR[4] = {0x00000000, 0x000000FF, 0x0000FFFF, 0x00FFFFFF};
            static const int shiftR[4] = {0, 8, 16, 24};
            u32 resultado = (base >> shiftR[alin]) | (reg & maskR[alin]);
            if (alin == 0) resultado = base;
            escribirGpr(RT(instr), resultado);
            break;
        }

        case 0x23: // LW
            {
                u32 rs = RS(instr);
                u32 rt = RT(instr);
                s32 immSE = (s32)(s16)IMM16(instr);
                u32 vaddr = cpu.gpr[rs] + immSE;
                cpu.gpr[rt] = memLeer32(vaddr);
            }
            break;

        case 0x21: // LH     rt = memoria[rs + inmediato]  (16 bits con signo)
            escribirGpr(RT(instr), (u32)(s32)(s16)memLeer16(cpu.gpr[RS(instr)] + IMM16_SEXT(instr)));
            break;

        case 0x25: // LHU    rt = memoria[rs + inmediato]  (16 bits sin signo)
            escribirGpr(RT(instr), memLeer16(cpu.gpr[RS(instr)] + IMM16_SEXT(instr)));
            break;

        case 0x20: // LB     rt = memoria[rs + inmediato]  (8 bits con signo)
            escribirGpr(RT(instr), (u32)(s32)(s8)memLeer8(cpu.gpr[RS(instr)] + IMM16_SEXT(instr)));
            break;

        case 0x24: // LBU    rt = memoria[rs + inmediato]  (8 bits sin signo)
            escribirGpr(RT(instr), memLeer8(cpu.gpr[RS(instr)] + IMM16_SEXT(instr)));
            break;

        case 0x2B: // SW     memoria[rs + inmediato] = rt  (32 bits)
            memEscribir32(cpu.gpr[RS(instr)] + IMM16_SEXT(instr), cpu.gpr[RT(instr)]);
            break;

        case 0x29: // SH     memoria[rs + inmediato] = rt  (16 bits)
            memEscribir16(cpu.gpr[RS(instr)] + IMM16_SEXT(instr), (u16)cpu.gpr[RT(instr)]);
            break;

        case 0x28: // SB     memoria[rs + inmediato] = rt  (8 bits)
            memEscribir8(cpu.gpr[RS(instr)] + IMM16_SEXT(instr), (u8)cpu.gpr[RT(instr)]);
            break;

            
        case 0x2A: { // SWL - Store Word Left
            u32 addr = cpu.gpr[RS(instr)] + IMM16_SEXT(instr);
            u32 alin = addr & 3;
            u32 base = memLeer32(addr & ~3);
            u32 reg  = cpu.gpr[RT(instr)];
            static const u32 mskSL[4] = {0x00FFFFFF, 0x0000FFFF, 0x000000FF, 0x00000000};
            static const int shSL[4]  = {24, 16, 8, 0};
            u32 resultado = (reg >> shSL[alin]) | (base & mskSL[alin]);
            memEscribir32(addr & ~3, resultado);
            break;
        }

        case 0x2E: { // SWR - Store Word Right
            u32 addr = cpu.gpr[RS(instr)] + IMM16_SEXT(instr);
            u32 alin = addr & 3;
            u32 base = memLeer32(addr & ~3);
            u32 reg  = cpu.gpr[RT(instr)];
            static const u32 mskSR[4] = {0x00000000, 0xFF000000, 0xFFFF0000, 0xFFFFFF00};
            static const int shSR[4]  = {0, 8, 16, 24};
            u32 resultado = (reg << shSR[alin]) | (base & mskSR[alin]);
            memEscribir32(addr & ~3, resultado);
            break;
        }

        case 0x2F: // CACHE - operacion de cache, no-op sin cache simulada
            break;

        case 0x39: { // SWC1  memoria[rs + imm] = fpr[rt]
            u32 addr = cpu.gpr[RS(instr)] + IMM16_SEXT(instr);
            u32 bits;
            memcpy(&bits, &g_fpr[RT(instr)], 4);
            memEscribir32(addr, bits);
            break;
        }

        case 0x31: { // LWC1  fpr[rt] = memoria[rs + imm]
            u32 addr = cpu.gpr[RS(instr)] + IMM16_SEXT(instr);
            u32 bits = memLeer32(addr);
            memcpy(&g_fpr[RT(instr)], &bits, 4);
            break;
        }

        case 0x18: { // VFPU - aritmetica de dos operandos: vadd/vsub/vdiv.
                     // Generalizado para .s/.p/.t/.q. Usa vfpuLeerFuente/
                     // vfpuEscribirDestino para respetar prefijos vpfxs/t/d
                     // si el juego los uso justo antes (swizzle/negacion/
                     // abs en las fuentes, mascara/saturacion en destino).
            u32 vd = instr & 0x7F;
            u32 vs = (instr >> 8) & 0x7F;
            u32 vt = (instr >> 16) & 0x7F;
            u32 tripleBit = (instr >> 15) & 1;
            u32 pairBit = (instr >> 7) & 1;
            u32 elementos = tripleBit ? (pairBit ? 4 : 3) : (pairBit ? 2 : 1);
            u32 firma = instr & 0xFF800000; // opcode + sub-operacion, sin tamano ni registros
            int manejado = 0;

            // vd = vs OP vt, elemento a elemento
            struct { u32 firma; int op; } binaria[] = {
                {0x60000000, 0}, // vadd
                {0x60800000, 1}, // vsub
                {0x63800000, 2}, // vdiv
            };
            for (u32 e = 0; e < sizeof(binaria)/sizeof(binaria[0]); e++) {
                if (firma == binaria[e].firma) {
                    for (u32 i = 0; i < elementos; i++) {
                        float a = vfpuLeerFuente(vs, elementos, i, g_pfxS.activo ? &g_pfxS : NULL);
                        float b = vfpuLeerFuente(vt, elementos, i, g_pfxT.activo ? &g_pfxT : NULL);
                        float r;
                        switch (binaria[e].op) {
                            case 0: r = a + b; break;
                            case 1: r = a - b; break;
                            case 2: r = (b != 0.0f) ? (a / b) : 0.0f; break;
                            default: r = 0.0f; break;
                        }
                        vfpuEscribirDestino(vd, elementos, i, r, g_pfxD.activo ? &g_pfxD : NULL);
                    }
                    manejado = 1;
                    break;
                }
            }

            vfpuConsumirPrefijos();

            if (!manejado) {
                logNoImplementado(instr, pcInstr, "VFPU opcode 0x18 variante desconocida (vadd/vsub/vdiv no matchearon)");
            }
            break;
        }

        case 0x19: { // VFPU - aritmetica de dos operandos: vmul/vdot/vscl (y a futuro
                     // vhdp/vcrs/vdet, que comparten este mismo opcode superior).
                     // OJO: antes estas tres estaban mal puestas en case 0x18 -
                     // el opcode real de vmul/vdot/vscl es 0x19 (primer byte
                     // 0x64-0x67), no 0x18 (primer byte 0x60-0x63). Nunca
                     // disparaban aunque el codigo estuviera bien escrito.
            u32 vd = instr & 0x7F;
            u32 vs = (instr >> 8) & 0x7F;
            u32 vt = (instr >> 16) & 0x7F;
            u32 tripleBit = (instr >> 15) & 1;
            u32 pairBit = (instr >> 7) & 1;
            u32 elementos = tripleBit ? (pairBit ? 4 : 3) : (pairBit ? 2 : 1);
            u32 firma = instr & 0xFF800000;
            int manejado = 0;

            if (firma == 0x64000000) { // vmul: vd = vs * vt, elemento a elemento
                for (u32 i = 0; i < elementos; i++) {
                    float a = vfpuLeerFuente(vs, elementos, i, g_pfxS.activo ? &g_pfxS : NULL);
                    float b = vfpuLeerFuente(vt, elementos, i, g_pfxT.activo ? &g_pfxT : NULL);
                    vfpuEscribirDestino(vd, elementos, i, a * b, g_pfxD.activo ? &g_pfxD : NULL);
                }
                manejado = 1;
            }

            if (!manejado && firma == 0x64800000) {
                // vdot %zs, %y_, %x_ - producto punto, resultado ESCALAR
                // (el destino es registro single sin importar el tamano de vs/vt;
                // el prefijo de destino no aplica aca, el hardware no lo usa
                // para resultados escalares de vdot)
                float suma = 0.0f;
                for (u32 i = 0; i < elementos; i++) {
                    float a = vfpuLeerFuente(vs, elementos, i, g_pfxS.activo ? &g_pfxS : NULL);
                    float b = vfpuLeerFuente(vt, elementos, i, g_pfxT.activo ? &g_pfxT : NULL);
                    suma += a * b;
                }
                memcpy(&cpu.cop2_regs[vfpuSingleIdx(vd)], &suma, 4);
                manejado = 1;
            }

            if (!manejado && firma == 0x65000000) {
                // vscl %z_, %y_, %xs - escala un vector por un escalar
                // (el ultimo operando es siempre registro single, sin prefijo)
                float escala;
                memcpy(&escala, &cpu.cop2_regs[vfpuSingleIdx(vt)], 4);
                for (u32 i = 0; i < elementos; i++) {
                    float a = vfpuLeerFuente(vs, elementos, i, g_pfxS.activo ? &g_pfxS : NULL);
                    vfpuEscribirDestino(vd, elementos, i, a * escala, g_pfxD.activo ? &g_pfxD : NULL);
                }
                manejado = 1;
            }

            vfpuConsumirPrefijos();

            if (!manejado) {
                logNoImplementado(instr, pcInstr, "VFPU opcode 0x19 variante desconocida (vcrs/vhdp/vdet/vqmul u otra)");
            }
            break;
        }

        case 0x34: { // VFPU - familia de operaciones de un vector (vmov/vabs/vneg/vsat/
                     // vrcp/vsqrt/vzero/vone). Generalizado para .s/.p/.t/.q igual que 0x18.
            u32 vd = instr & 0x7F;
            u32 vs = (instr >> 8) & 0x7F;
            u32 tripleBit = (instr >> 15) & 1;
            u32 pairBit = (instr >> 7) & 1;
            u32 elementos = tripleBit ? (pairBit ? 4 : 3) : (pairBit ? 2 : 1);
            u32 firma = instr & 0xFFFF0000; // opcode + sub-operacion, sin tamano ni registros
            int manejado = 0;

            struct { u32 firma; int op; } elementwise[] = {
                {0xD0000000, 0}, // vmov  : tal cual
                {0xD0010000, 1}, // vabs  : valor absoluto
                {0xD0020000, 2}, // vneg  : negacion
                {0xD0040000, 3}, // vsat0 : clamp [0,1]
                {0xD0050000, 4}, // vsat1 : clamp [-1,1]
                {0xD0100000, 5}, // vrcp  : 1/x
                {0xD0110000, 7}, // vrsq  : 1/sqrt(x)
                {0xD0160000, 6}, // vsqrt : raiz cuadrada
            };

            for (u32 e = 0; e < sizeof(elementwise)/sizeof(elementwise[0]); e++) {
                if (firma == elementwise[e].firma) {
                    for (u32 i = 0; i < elementos; i++) {
                        float val = vfpuLeerFuente(vs, elementos, i, g_pfxS.activo ? &g_pfxS : NULL);
                        switch (elementwise[e].op) {
                            case 0: break; // vmov
                            case 1: val = fabsf(val); break;
                            case 2: val = -val; break;
                            case 3: val = val < 0.0f ? 0.0f : (val > 1.0f ? 1.0f : val); break;
                            case 4: val = val < -1.0f ? -1.0f : (val > 1.0f ? 1.0f : val); break;
                            case 5: val = (val != 0.0f) ? (1.0f / val) : 0.0f; break;
                            case 6: val = sqrtf(val); break;
                            case 7: val = (val > 0.0f) ? (1.0f / sqrtf(val)) : 0.0f; break;
                        }
                        vfpuEscribirDestino(vd, elementos, i, val, g_pfxD.activo ? &g_pfxD : NULL);
                    }
                    manejado = 1;
                    break;
                }
            }

            if (!manejado) {
                float valorConst = 0.0f;
                int esConst = 0;
                if (firma == 0xD0060000) { esConst = 1; valorConst = 0.0f; } // vzero
                else if (firma == 0xD0070000) { esConst = 1; valorConst = 1.0f; } // vone

                if (esConst) {
                    for (u32 i = 0; i < elementos; i++) {
                        vfpuEscribirDestino(vd, elementos, i, valorConst, g_pfxD.activo ? &g_pfxD : NULL);
                    }
                    manejado = 1;
                }
            }

            if (!manejado && firma == 0xD0030000) {
                // vidt: version "un vector" de vmidt (identidad de matriz).
                // Pone 1.0 en la posicion que coincide con la columna/fila
                // propia de vd, y 0.0 en el resto - construye la fila o
                // columna de una matriz identidad sin tocar el resto.
                u32 posIdentidad = vd & 3;
                u32 offVd = vfpuOffsetTamano(vd, elementos);
                for (u32 i = 0; i < elementos; i++) {
                    float val = ((offVd + i) == posIdentidad) ? 1.0f : 0.0f;
                    vfpuEscribirDestino(vd, elementos, i, val, g_pfxD.activo ? &g_pfxD : NULL);
                }
                manejado = 1;
            }

            vfpuConsumirPrefijos();

            if (!manejado) {
                logNoImplementado(instr, pcInstr, "VFPU opcode 0x34 variante desconocida (familia vabs/vneg/vsat/vrcp/vsqrt/vzero/vone/vidt)");
            }
            break;
        }

        case 0x3C: { // VFPU - varias instrucciones de matriz comparten este opcode
                     // superior (vmidt/vmzero/vmone/vmmov/vmmul/etc), se distinguen
                     // por los bits fijos del resto de la instruccion.
            u32 reg = instr & 0x7F;
            u32 banco = (reg >> 2) & 7;        // cual de los 8 bancos de 4x4

            if ((instr & 0xFFFFFF80) == 0xF3838080) { // vmidt.q %zo - matriz identidad 4x4
                for (u32 col = 0; col < 4; col++) {
                    for (u32 fila = 0; fila < 4; fila++) {
                        float valor = (col == fila) ? 1.0f : 0.0f;
                        memcpy(&cpu.cop2_regs[vfpuIdx(reg, col, fila)], &valor, 4);
                    }
                }
            } else if ((instr & 0xFFFFFF80) == 0xF3868080) { // vmzero.q %zo - matriz en cero
                for (u32 i = 0; i < 16; i++) {
                    cpu.cop2_regs[banco*32 + i] = 0;
                }
            } else if ((instr & 0xFF808080) == 0xF0008080) { // vmmul.q %zo, %yo, %xo - multiplica matrices 4x4
                u32 vd = instr & 0x7F;
                u32 vs = (instr >> 8) & 0x7F;
                u32 vt = (instr >> 16) & 0x7F;
                float resultado[4][4];

                for (u32 col = 0; col < 4; col++) {
                    for (u32 fila = 0; fila < 4; fila++) {
                        float suma = 0.0f;
                        for (u32 k = 0; k < 4; k++) {
                            float a, b;
                            memcpy(&a, &cpu.cop2_regs[vfpuIdx(vs, k, fila)], 4);
                            memcpy(&b, &cpu.cop2_regs[vfpuIdx(vt, col, k)], 4);
                            suma += a * b;
                        }
                        resultado[col][fila] = suma;
                    }
                }
                // Copiar al final (no en el loop de arriba): vd puede ser el
                // mismo registro que vs o vt, no hay que pisar mientras leemos.
                for (u32 col = 0; col < 4; col++) {
                    for (u32 fila = 0; fila < 4; fila++) {
                        memcpy(&cpu.cop2_regs[vfpuIdx(vd, col, fila)], &resultado[col][fila], 4);
                    }
                }
            } else if ((instr & 0xFF808080) == 0xF0800080 ||  // vtfm2.p %zp, %ym, %xp
                       (instr & 0xFF808080) == 0xF1008000 ||  // vtfm3.t %zt, %yn, %xt
                       (instr & 0xFF808080) == 0xF1808080) {  // vtfm4.q %zq, %yo, %xq
                // Transforma un vector NxN por una matriz NxN (N=2,3,4 segun tamano).
                // No confundir con vmmul: aca el segundo operando es un VECTOR, no
                // otra matriz completa.
                u32 vd = instr & 0x7F;
                u32 vs = (instr >> 8) & 0x7F;  // matriz NxN
                u32 vt = (instr >> 16) & 0x7F; // vector fuente

                u32 n = 4;
                if ((instr & 0xFF808080) == 0xF0800080) n = 2;
                else if ((instr & 0xFF808080) == 0xF1008000) n = 3;

                u32 offVd = (n == 2) ? (((vd>>6)&1) ? 2 : 0) : (n == 3) ? (((vd>>6)&1) ? 1 : 0) : 0;
                u32 offVt = (n == 2) ? (((vt>>6)&1) ? 2 : 0) : (n == 3) ? (((vt>>6)&1) ? 1 : 0) : 0;

                float entrada[4], salida[4];
                for (u32 i = 0; i < n; i++) {
                    memcpy(&entrada[i], &cpu.cop2_regs[vfpuIdx(vt, vt & 3, offVt + i)], 4);
                }
                for (u32 fila = 0; fila < n; fila++) {
                    float suma = 0.0f;
                    for (u32 col = 0; col < n; col++) {
                        float m;
                        memcpy(&m, &cpu.cop2_regs[vfpuIdx(vs, col, fila)], 4); // matriz completa, sin offset
                        suma += m * entrada[col];
                    }
                    salida[fila] = suma;
                }
                for (u32 i = 0; i < n; i++) {
                    memcpy(&cpu.cop2_regs[vfpuIdx(vd, vd & 3, offVd + i)], &salida[i], 4);
                }
            } else {
                logNoImplementado(instr, pcInstr, "VFPU opcode 0x3C variante desconocida (familia vmidt/vmzero/vmone/vmmov/vmmul/vtfm/vhtfm)");
            }
            break;
        }

        case 0x37: { // vpfxs / vpfxt / vpfxd - prefijos VFPU. Se distinguen
                     // por el byte superior completo (mascara 0xFF000000).
            u32 topByte = instr & 0xFF000000;
            if (topByte == 0xDC000000) {
                vfpuParsearPrefijoFuente(instr, &g_pfxS);
            } else if (topByte == 0xDD000000) {
                vfpuParsearPrefijoFuente(instr, &g_pfxT);
            } else if (topByte == 0xDE000000) {
                vfpuParsearPrefijoDestino(instr, &g_pfxD);
            } else {
                // 0xDF___ es viim.s/vfim.s (carga inmediato en un registro
                // VFPU) - distinto de los prefijos, todavia no implementado.
                logNoImplementado(instr, pcInstr, "VFPU opcode 0x37 variante desconocida (viim.s/vfim.s u otra)");
            }
            break;
        }

        case 0x36: { // lv.q vt, offset(rs) - carga un vector de 4 registros VFPU
                     // (fila o columna de una matriz 4x4) desde memoria. Misma
                     // codificacion que sv.q (ver arriba), pero de lectura.
            u32 rs = RS(instr);
            u32 vtLow5 = RT(instr);          // bits 20-16
            u32 vtHigh2 = instr & 0x3;       // bits 1-0
            u32 vt = (vtHigh2 << 5) | vtLow5; // registro VFPU de 7 bits

            s32 offsetRaw = (s32)((instr >> 2) & 0x3FFF); // 14 bits
            if (offsetRaw & 0x2000) offsetRaw -= 0x4000;  // signo
            u32 addr = cpu.gpr[rs] + (u32)(offsetRaw * 4);

            u32 banco = (vt >> 2) & 7;
            u32 fijo = vt & 3;
            u32 transpuesta = (vt >> 5) & 1;

            for (u32 i = 0; i < 4; i++) {
                u32 idx = transpuesta ? (banco*32 + i*4 + fijo) : (banco*32 + fijo*4 + i);
                cpu.cop2_regs[idx] = memLeer32(addr + i*4);
            }
            break;
        }

        case 0x3E: { // sv.q / vwb.q - comparten opcode superior, se distinguen por el bit 1
            if ((instr & 0x2) != 0) {
                logNoImplementado(instr, pcInstr, "VFPU vwb.q no implementada (comparte opcode con sv.q)");
                break;
            }
            // sv.q: guarda un vector de 4 registros VFPU (fila o columna de
            // una matriz 4x4) en memoria. Direccion debe ser 16 bytes alineada.
            u32 rs = RS(instr);
            u32 vtLow5 = RT(instr);          // bits 20-16
            u32 vtHigh2 = instr & 0x3;       // bits 1-0
            u32 vt = (vtHigh2 << 5) | vtLow5; // registro VFPU de 7 bits

            s32 offsetRaw = (s32)((instr >> 2) & 0x3FFF); // 14 bits
            if (offsetRaw & 0x2000) offsetRaw -= 0x4000;  // signo
            u32 addr = cpu.gpr[rs] + (u32)(offsetRaw * 4);

            u32 banco = (vt >> 2) & 7;
            u32 fijo = vt & 3;
            u32 transpuesta = (vt >> 5) & 1;

            for (u32 i = 0; i < 4; i++) {
                u32 idx = transpuesta ? (banco*32 + i*4 + fijo) : (banco*32 + fijo*4 + i);
                memEscribir32(addr + i*4, cpu.cop2_regs[idx]);
            }
            break;
        }

        default:
            logNoImplementado(instr, pcInstr, "opcode desconocido (tipo I/J no manejado aca)");
            break;
    }
}

// ============================================================
// Detectar y resolver instrucciones de SALTO (branches/jumps).
// Devuelve 1 si 'instr' es un salto, y en ese caso llena
// *tomado y *destino. Devuelve 0 si no es un salto (para que
// el llamador la ejecute como instruccion normal).
// ============================================================
static int esSaltoYResolver(u32 instr, u32 pcInstr, int *tomado, u32 *destino, int *esLikely) {
    u32 opcode = OPCODE(instr);
    *esLikely = 0;

    switch (opcode) {

        case 0x01: { // REGIMM: BLTZ, BGEZ, BLTZL, BGEZL, BLTZAL, BGEZAL
            u32 selector = RT(instr); // ojo: aca NO es un registro, es el sub-opcode

            switch (selector) {
                case 0x00: // BLTZ    if (rs < 0) salta
                    *tomado = ((s32)cpu.gpr[RS(instr)] < 0);
                    *destino = pcInstr + 4 + (IMM16_SEXT(instr) << 2);
                    return 1;

                case 0x01: // BGEZ    if (rs >= 0) salta
                    *tomado = ((s32)cpu.gpr[RS(instr)] >= 0);
                    *destino = pcInstr + 4 + (IMM16_SEXT(instr) << 2);
                    return 1;

                case 0x02: // BLTZL   (variante "likely")
                    *esLikely = 1;
                    *tomado = ((s32)cpu.gpr[RS(instr)] < 0);
                    *destino = pcInstr + 4 + (IMM16_SEXT(instr) << 2);
                    return 1;

                case 0x03: // BGEZL   (variante "likely")
                    *esLikely = 1;
                    *tomado = ((s32)cpu.gpr[RS(instr)] >= 0);
                    *destino = pcInstr + 4 + (IMM16_SEXT(instr) << 2);
                    return 1;

                case 0x10: // BLTZAL  if (rs < 0) salta y guarda $ra
                    *tomado = ((s32)cpu.gpr[RS(instr)] < 0);
                    *destino = pcInstr + 4 + (IMM16_SEXT(instr) << 2);
                    escribirGpr(31, pcInstr + 8);
                    return 1;

                case 0x11: // BGEZAL  if (rs >= 0) salta y guarda $ra
                    *tomado = ((s32)cpu.gpr[RS(instr)] >= 0);
                    *destino = pcInstr + 4 + (IMM16_SEXT(instr) << 2);
                    escribirGpr(31, pcInstr + 8);
                    return 1;

                case 0x14: { // BEQL
                    u32 rs = RS(instr);
                    u32 rt = RT(instr);
                    s32 immSE = IMM16_SEXT(instr);
                    *esLikely = 1;
                    *tomado = (cpu.gpr[rs] == cpu.gpr[rt]);
                    *destino = pcInstr + 4 + (immSE << 2);
                    return 1;
                }

                default:
                    return 0;
            }
        }

        case 0x02: // J        salto incondicional absoluto (26 bits)
            *tomado = 1;
            *destino = (pcInstr & 0xF0000000) | (DIR26(instr) << 2);
            return 1;

        case 0x03: // JAL      salto absoluto + guarda direccion de retorno en $ra
            *tomado = 1;
            *destino = (pcInstr & 0xF0000000) | (DIR26(instr) << 2);
            escribirGpr(31, pcInstr + 8); // $ra = instruccion siguiente al delay slot
            return 1;

        case 0x04: // BEQ      if (rs == rt) salta
            *tomado = (cpu.gpr[RS(instr)] == cpu.gpr[RT(instr)]);
            *destino = pcInstr + 4 + (IMM16_SEXT(instr) << 2);
            return 1;

        case 0x05: // BNE      if (rs != rt) salta
            *tomado = (cpu.gpr[RS(instr)] != cpu.gpr[RT(instr)]);
            *destino = pcInstr + 4 + (IMM16_SEXT(instr) << 2);
            return 1;

        case 0x14: // BEQL
            {
                u32 rs = RS(instr);
                u32 rt = RT(instr);
                s32 immSE = (s32)(s16)IMM16(instr);
                *esLikely = 1;
                *tomado = (cpu.gpr[rs] == cpu.gpr[rt]);
                *destino = pcInstr + 4 + (immSE << 2);
            }
            return 1;

        case 0x15: // BNEL
            *esLikely = 1;
            *tomado = (cpu.gpr[RS(instr)] != cpu.gpr[RT(instr)]);
            *destino = pcInstr + 4 + (IMM16_SEXT(instr) << 2);
            return 1;

        case 0x16: // BLEZL
            *esLikely = 1;
            *tomado = ((s32)cpu.gpr[RS(instr)] <= 0);
            *destino = pcInstr + 4 + (IMM16_SEXT(instr) << 2);
            return 1;

        case 0x17: // BGTZL
            *esLikely = 1;
            *tomado = ((s32)cpu.gpr[RS(instr)] > 0);
            *destino = pcInstr + 4 + (IMM16_SEXT(instr) << 2);
            return 1;

        case 0x06: // BLEZ     if (rs <= 0) salta
            *tomado = ((s32)cpu.gpr[RS(instr)] <= 0);
            *destino = pcInstr + 4 + (IMM16_SEXT(instr) << 2);
            return 1;

        case 0x07: // BGTZ     if (rs > 0) salta
            *tomado = ((s32)cpu.gpr[RS(instr)] > 0);
            *destino = pcInstr + 4 + (IMM16_SEXT(instr) << 2);
            return 1;

        case 0x00: {
            u32 funct = FUNCT(instr);
            if (funct == 0x08) { // JR    salta a la direccion en rs
                *tomado = 1;
                u32 vaddrDest = cpu.gpr[RS(instr)];
                // Mapear vaddr -> real si cae en rango virtual de algun modulo
                // O si ya es una direccion real en el rango real, usarla directo
                // NO mapear si vaddrDest == 0 (NULL): es inválido como puntero de código
                *destino = vaddrDest;
                if (vaddrDest != 0) {
                    for (int i = 0; i < g_cantidadModulos; i++) {
                        if (!g_modulos[i].activo) continue;
                        if (vaddrDest >= g_modulos[i].vaddrMin &&
                            vaddrDest < g_modulos[i].vaddrMax) {
                            // Es una virtual address: convertir a real
                            s32 delta = (s32)g_modulos[i].realMin - (s32)g_modulos[i].vaddrMin;
                            *destino = vaddrDest + delta;
                            break;
                        }
                        if (vaddrDest >= g_modulos[i].realMin &&
                            vaddrDest < g_modulos[i].realMax) {
                            // Ya es una real address dentro del modulo: usar tal cual
                            *destino = vaddrDest;
                            break;
                        }
                    }
                }
                return 1;
            }
            if (funct == 0x09) { // JALR  salta a rs, guarda retorno en rd
                *tomado = 1;
                u32 vaddrDest = cpu.gpr[RS(instr)];
                *destino = vaddrDest;
                if (vaddrDest != 0) {
                    for (int i = 0; i < g_cantidadModulos; i++) {
                        if (!g_modulos[i].activo) continue;
                        if (vaddrDest >= g_modulos[i].vaddrMin &&
                            vaddrDest < g_modulos[i].vaddrMax) {
                            s32 delta = (s32)g_modulos[i].realMin - (s32)g_modulos[i].vaddrMin;
                            *destino = vaddrDest + delta;
                            break;
                        }
                        if (vaddrDest >= g_modulos[i].realMin &&
                            vaddrDest < g_modulos[i].realMax) {
                            *destino = vaddrDest;
                            break;
                        }
                    }
                }
                escribirGpr(RD(instr) ? RD(instr) : 31, pcInstr + 8);
                return 1;
            }
            return 0;
        }
        default:
            return 0;
    }
}

// ============================================================
// Bucle principal del interprete
// ============================================================
// (PC_CALLBACK_SALIDA ahora se define arriba, junto a PC_HILO_SALIDA)

static u32 g_ultimoPcNoCero = 0;
static u32 g_ultimaInstrNoCero = 0;
// (g_enInvocacionCallback ahora se declara arriba, junto a logNoImplementado,
// porque logNoImplementado la necesita)

// Ejecuta UNA instruccion del hilo actual (con su delay slot si es un
// salto), avanza pc, cuenta la instruccion, y cede el CPU a otro hilo
// si el actual quedo bloqueado. Es el cuerpo de un paso del interprete,
// factorizado para poder reusarlo tanto en el loop principal como
// dentro de invocarFuncionPSP (para poder "llamar" funciones PSP reales
// desde C, como un handler de VBLANK).
// Una direccion de PC es valida si cae dentro del rango del EBOOT principal
// (g_vaddrMinimo/Max) o de cualquier modulo cargado (kjfs, etc.).
static int direccionValida(u32 pc) {
    if (pc >= g_vaddrMinimo && pc < g_vaddrMaximo) return 1;
    for (int i = 0; i < g_cantidadModulos; i++) {
        if (g_modulos[i].activo &&
            pc >= g_modulos[i].realMin && pc < g_modulos[i].realMax)
            return 1;
    }
    return 0;
}

static void ejecutarUnPaso(void) {
    u32 pcActual = cpu.pc;

    if (pcActual == PC_HILO_SALIDA) {
        g_hilos[g_hiloActual].estado = HILO_TERMINADO;
        printf("TRADUCTOR: hilo uid=0x%08X termino\n", (unsigned int)g_hilos[g_hiloActual].uid);
        if (!planificar()) cpu.detenido = 1;
        return;
    }

    if (pcActual == PC_MODULO_RET) {
        // Una funcion de modulo devolvio: restaurar el $gp del llamador y
        // volver a quien nos llamo (direccion guardada en la pila de frames).
        if (g_spModulo > 0) {
            g_spModulo--;
            cpu.gpr[28] = g_pilaModulo[g_spModulo].caller_gp;
            cpu.pc      = g_pilaModulo[g_spModulo].return_addr;
        } else {
            printf("TRADUCTOR: PC_MODULO_RET sin frames en la pila (spModulo=0)\n");
            cpu.pc = PC_SALIDA_MODULO;
        }
        return;
    }

    if (!direccionValida(pcActual)) {
        printf(
            "TRADUCTOR: ultimo codigo no-cero visto:\n"
            "  pc       = 0x%08X\n"
            "  instr    = 0x%08X\n",
            (unsigned int)g_ultimoPcNoCero, (unsigned int)g_ultimaInstrNoCero
        );
        printf(
            "TRADUCTOR: PC FUERA DE TODO MODULO (hilo uid=0x%08X termina)\n"
            "  pc       = 0x%08X\n"
            "  rangos   = EBOOT 0x%08X-0x%08X | %d modulo(s)\n"
            "  pasos    = %u\n",
            (unsigned int)g_hilos[g_hiloActual].uid, (unsigned int)pcActual,
            (unsigned int)g_vaddrMinimo, (unsigned int)g_vaddrMaximo,
            (unsigned int)g_cantidadModulos,
            (unsigned int)g_contadorInstrucciones
        );
        // Dump de registros para cazar el puntero de funcion mal relocalizado
        // que dejo $ra en una vaddr en vez de una direccion real.
        printf("  REGS: ra=0x%08X t9=0x%08X gp=0x%08X sp=0x%08X v0=0x%08X v1=0x%08X\n",
            (unsigned int)cpu.gpr[31], (unsigned int)cpu.gpr[25],
            (unsigned int)cpu.gpr[28], (unsigned int)cpu.gpr[29],
            (unsigned int)cpu.gpr[2], (unsigned int)cpu.gpr[3]);
        printf("  REGS: a0=0x%08X a1=0x%08X a2=0x%08X a3=0x%08X t0=0x%08X t1=0x%08X\n",
            (unsigned int)cpu.gpr[4], (unsigned int)cpu.gpr[5], (unsigned int)cpu.gpr[6],
            (unsigned int)cpu.gpr[7], (unsigned int)cpu.gpr[8], (unsigned int)cpu.gpr[9]);
        // Ultimas 4 instrucciones antes del salto (cada una es un word de 32 bits).
        if (g_ultimoPcNoCero >= 0x10) {
            for (int d = 3; d >= 0; d--) {
                u32 pa = g_ultimoPcNoCero - d*4;
                u32 wi = memLeer32(pa);
                printf("    [%d] 0x%08X: 0x%08X\n", d, (unsigned int)pa, (unsigned int)wi);
            }
        }
        g_hilos[g_hiloActual].estado = HILO_TERMINADO;
        if (!planificar()) cpu.detenido = 1;
        return;
    }

    u32 instr = memLeer32(pcActual);

    if (g_trazaHiloNuevo > 0) {
        printf("TRAZA HILO uid=0x%08X pc=0x%08X instr=0x%08X sp=0x%08X ra=0x%08X\n",
            (unsigned int)g_hilos[g_hiloActual].uid, (unsigned int)pcActual,
            (unsigned int)instr, (unsigned int)cpu.gpr[29], (unsigned int)cpu.gpr[31]);
        g_trazaHiloNuevo--;
    }

    if (instr != 0x00000000) {
        g_ultimoPcNoCero = pcActual;
        g_ultimaInstrNoCero = instr;
    }

    int esSalto, tomado = 0, esLikely = 0;
    u32 destino = 0;
    esSalto = esSaltoYResolver(instr, pcActual, &tomado, &destino, &esLikely);

    if (esSalto) {
        u32 pcDelay = pcActual + 4;
        if (!esLikely || tomado) {
            g_pcForzadoPorImport = 0;
            u32 instrDelay = memLeer32(pcDelay);
            ejecutarNormal(instrDelay, pcDelay);
        }
        if (g_pcForzadoPorImport) {
            g_pcForzadoPorImport = 0;
        } else {
            cpu.pc = tomado ? destino : (pcActual + 8);
        }
    } else {
        ejecutarNormal(instr, pcActual);
        if (g_pcForzadoPorImport) {
            g_pcForzadoPorImport = 0;
        } else {
            cpu.pc = pcActual + 4;
        }
    }

    g_contadorInstrucciones++;

    // Punto de cesion del scheduler cooperativo. Se salta si estamos
    // adentro de una invocacion de callback (invocarFuncionPSP) - no
    // queremos que el scheduler le robe el CPU a la funcion que
    // estamos tratando de correr hasta que termine.
    if (!g_enInvocacionCallback && g_hilos[g_hiloActual].estado != HILO_LISTO) {
        if (!planificar()) cpu.detenido = 1;
    }

    // SendMsgPipe encolo un mensaje y quiere ceder el CPU para que el hilo
    // worker/receptor pueda correr y vaciar la cola. Solo cedemos si hay otro
    // hilo LISTO (si no, no hay a quien cederle y seguimos).
    if (!g_enInvocacionCallback && g_forzarCesion) {
        g_forzarCesion = 0;
        if (g_hilos[g_hiloActual].estado == HILO_LISTO &&
            hilo_buscarSiguienteListo() >= 0) {
            planificar();
        }
    }
}

// Llama a una funcion PSP real (por direccion) y espera a que vuelva,
// ejecutando instrucciones de verdad con nuestro propio interprete.
// Usa un $ra centinela (PC_CALLBACK_SALIDA) para saber cuando volvio.
// Guarda y restaura TODO el estado de CPU antes/despues - para el
// resto del sistema es invisible que esto paso (salvo los efectos
// secundarios reales que la funcion haya hecho sobre memoria/hilos,
// que SI quedan, es la forma de que el callback haga algo util).
static void invocarFuncionPSP(u32 direccionFuncion, u32 a0, u32 a1, u32 a2, u32 a3) {
    if (direccionFuncion == 0) return; // handler nulo, nada que llamar
    if (!direccionValida(direccionFuncion)) return; // direccion invalida

    CpuSimulada guardado = cpu;

    cpu.gpr[4] = a0;
    cpu.gpr[5] = a1;
    cpu.gpr[6] = a2;
    cpu.gpr[7] = a3;
    cpu.gpr[31] = PC_CALLBACK_SALIDA;
    cpu.pc = direccionFuncion;

    int enInvocacionAnterior = g_enInvocacionCallback;
    g_enInvocacionCallback = 1;

    const int MAX_PASOS_CALLBACK = 2000000; // salvavidas, no deberia hacer falta
    int pasos = 0;
    while (cpu.pc != PC_CALLBACK_SALIDA && !cpu.detenido && pasos < MAX_PASOS_CALLBACK) {
        ejecutarUnPaso();
        pasos++;
    }

    if (pasos >= MAX_PASOS_CALLBACK) {
        printf("TRADUCTOR: invocarFuncionPSP - la funcion 0x%08X nunca volvio (limite de seguridad)\n",
            (unsigned int)direccionFuncion);
    }

    g_enInvocacionCallback = enInvocacionAnterior;
    cpu = guardado; // restaurar registros/pc/estado del hilo que llamo a esto
}

// Dispara un "tick" de VBLANK si paso suficiente tiempo real (~1/60s,
// NTSC). En un tick real: invoca cualquier handler de sub-interrupcion
// registrado y habilitado, Y ademas TODOS los callbacks creados via
// sceKernelCreateCallback (no sabemos con certeza cual de los que
// registra el juego es especificamente "el de vblank", asi que
// disparamos todos - es una aproximacion, no el comportamiento exacto
// de hardware, pero deberia destrabar lo que este esperando alguna
// notificacion periodica).
static clock_t g_ultimoVblank = 0;

static const char *nombreEstadoHilo(EstadoHilo e) {
    switch (e) {
        case HILO_LIBRE: return "LIBRE";
        case HILO_CREADO: return "CREADO";
        case HILO_LISTO: return "LISTO";
        case HILO_DURMIENDO: return "DURMIENDO";
        case HILO_ESPERANDO_SEMA: return "ESPERANDO_SEMA";
        case HILO_TERMINADO: return "TERMINADO";
        default: return "?";
    }
}

static void volcarEstadoHilos(void) {
    printf("DEBUG ESTADO: %d hilos (actual=uid 0x%08X)\n",
        g_cantidadHilos, (unsigned int)g_hilos[g_hiloActual].uid);
    for (int i = 0; i < g_cantidadHilos; i++) {
        Hilo *h = &g_hilos[i];
        CpuSimulada *hc = &g_hilos_cpu[i];
        printf("  hilo[%d] uid=0x%08X estado=%s pc=0x%08X",
            i, (unsigned int)h->uid, nombreEstadoHilo(h->estado), (unsigned int)hc->pc);
        if (h->estado == HILO_ESPERANDO_SEMA) {
            printf(" esperando_sema=%u cantidad=%d sema_actual=%d",
                (unsigned int)h->semaEsperado, h->semaCantidad,
                (h->semaEsperado < MAX_SEMAFOROS) ? g_semaCount[h->semaEsperado] : -1);
        }
        printf("\n");
    }
}

static int g_vecesVblank = 0; // debug temporal

static void chequearVblank(void) {
    clock_t ahora = clock();
    if (g_ultimoVblank == 0) { g_ultimoVblank = ahora; return; }

    double segundosPasados = (double)(ahora - g_ultimoVblank) / CLOCKS_PER_SEC;
    if (segundosPasados < (1.0 / 60.0)) return;
    g_ultimoVblank = ahora;

    g_vecesVblank++;
    int mostrarDetalle = (g_vecesVblank <= 30);
    if (mostrarDetalle) {
        printf("DEBUG VBLANK #%d: %d subIntr, %u callbacks\n",
            g_vecesVblank, g_cantidadSubIntr, (unsigned int)g_cantidadCallbacks);
    }

    for (int i = 0; i < g_cantidadSubIntr; i++) {
        if (g_subIntr[i].habilitado && g_subIntr[i].handler != 0) {
            if (mostrarDetalle) printf("DEBUG VBLANK: invocando subIntr[%d] handler=0x%08X\n", i, g_subIntr[i].handler);
            invocarFuncionPSP(g_subIntr[i].handler, g_subIntr[i].subIntr, g_subIntr[i].common, 0, 0);
            if (mostrarDetalle) printf("DEBUG VBLANK: subIntr[%d] volvio OK\n", i);
        }
    }
    for (u32 i = 0; i < g_cantidadCallbacks; i++) {
        if (g_callbacks[i].func != 0) {
            if (mostrarDetalle) printf("DEBUG VBLANK: invocando callback uid=0x%08X func=0x%08X\n",
                (unsigned int)g_callbacks[i].uid, (unsigned int)g_callbacks[i].func);
            // Firma tipica de un callback PSP: (int count, int arg, void *common)
            invocarFuncionPSP(g_callbacks[i].func, 1, 0, g_callbacks[i].arg, 0);
            if (mostrarDetalle) printf("DEBUG VBLANK: callback uid=0x%08X volvio OK\n", (unsigned int)g_callbacks[i].uid);
        }
    }

    if (mostrarDetalle) volcarEstadoHilos();
}

void traductor_ejecutar(u32 puntoDeEntradaVirtual) {
    memset(g_hilos, 0, sizeof(g_hilos));
    g_hilos[0].uid = 0x10000; // hilo principal, uid fijo (nunca lo pide el juego por CreateThread)
    g_hilos[0].estado = HILO_LISTO;
    g_cantidadHilos = 1;
    g_hiloActual = 0;

    for (int i = 0; i < 32; i++) 
    {
        cpu.gpr[i] = g_hayRegistrosIniciales[i] ? g_registrosIniciales[i] : 0;
    }
    cpu.pc = puntoDeEntradaVirtual;
    cpu.hi = cpu.lo = 0;
    cpu.detenido = 0;
    memset(cpu.cop2_regs, 0, sizeof(cpu.cop2_regs));

    printf("TRADUCTOR: arrancando interprete en vaddr=0x%08X\n", (unsigned int)puntoDeEntradaVirtual);
    printf("TRADUCTOR: cantBloquesMem=%u\n", (unsigned int)g_cantidadBloques);

    // Arrancar los modulos PSP adicionales cargados (ej: kjfs) ejecutando
    // su module_start, ANTES de correr el juego. Asi quedan inicializados
    // (globales, tablas) y sus hilos (si los crean) conviven con el hilo
    // principal en g_hilos. Se hace aca y no en el loader para no perder
    // esos hilos cuando se arma el hilo 0.
    for (int m = 0; m < g_cantidadModulos; m++) {
        traductor_iniciarModulo(m);
    }

    g_contadorInstrucciones = 0;

    while (!cpu.detenido) {
        ejecutarUnPaso();

        // Chequear VBLANK cada ~16384 instrucciones.
        // chequearVblank() ya usa clock() internamente para ver si paso ~1/60s real.
        if ((g_contadorInstrucciones & 0x3FFF) == 0) {
            chequearVblank();
            // VBLANK corre callbacks via invocarFuncionPSP que restaura cpu desde un backup.
            // Debemos guardar el estado actualizado en el slot del hilo para que persista.
            if (!g_enInvocacionCallback && g_hiloActual >= 0 && g_hiloActual < g_cantidadHilos) {
                g_hilos_cpu[g_hiloActual] = cpu;
            }
        }
        // Despues de VBLANK: los callbacks pueden haber despertado hilos o creado nuevos.
        // Si hay OTRO hilo LISTO (distinto del actual), ceder el CPU (round-robin).
        if (!g_enInvocacionCallback && g_hilos[g_hiloActual].estado == HILO_LISTO) {
            int siguiente = hilo_buscarSiguienteListo();
            if (siguiente >= 0 && siguiente != g_hiloActual) {
                planificar();
            }
        }
    }
    printf("TRADUCTOR: detenido despues de %u instrucciones. pc final=0x%08X\n",
           (unsigned int)g_contadorInstrucciones, (unsigned int)cpu.pc);
}