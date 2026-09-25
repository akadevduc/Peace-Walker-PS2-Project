#ifndef TRADUCTOR_H
#define TRADUCTOR_H

#include <tamtypes.h>
#include <dirent.h>

#ifdef __cplusplus
extern "C" {
#endif

// Le informa al traductor cual es el delta (real - virtual) que hay
// que sumarle a cualquier direccion del modulo PSP para encontrarla
// en la memoria real de la PS2. Llamar esto UNA vez, despues de
// terminar de cargar y relocalizar el modulo, y ANTES de traductor_ejecutar.
void traductor_setDelta(s32 delta);

#define TRADUCTOR_MAX_SEGMENTOS 16

void traductor_setSegmentosArchivo(
    const u32 *vaddrSegmentos,
    const u32 *offsetSegmentos,
    const u32 *fileszSegmentos,
    int cantidadSegmentos
);

#define PC_SALIDA_MODULO 0xFFFFFFFF

// Sentinels de retorno usados por el bucle del interprete para saber cuando
// un hilo / callback / modulo terminaron y deben volver al llamador.
#define PC_HILO_SALIDA 0xFFFFFFFE     // distinto de PC_SALIDA_MODULO
#define PC_CALLBACK_SALIDA 0xFFFFFFFD // distinto de los otros dos (invocarFuncionPSP)

void traductor_setRegistroInicial(int indice, u32 valor);

void traductor_setRangoVirtual(u32 minimo, u32 maximo);

void traductor_setPartitionPool(u8 *pool, u32 size);

// Directorio base en el host donde esta la carpeta del ISO/UMD extraido
// (la que tiene PSP_GAME/USRDIR/etc adentro). sceIoOpen usa esto para
// traducir rutas PSP reales (disc0:/algo, fatms0:/algo, etc) a archivos
// de verdad. Llamar ANTES de traductor_ejecutar.
void traductor_setDirectorioDatos(const char *ruta);

// Arranca el interprete en la direccion VIRTUAL ORIGINAL del PSP (la
// que trae header.e_entry, ej: 0x114 - NO la ya relocalizada/real).
// El traductor internamente le suma 'delta' cada vez que necesita
// tocar memoria; si le pasas una direccion ya relocalizada, el delta
// se sumaria dos veces y todo se rompe.
void traductor_ejecutar(u32 puntoDeEntradaVirtual);

#define TRADUCTOR_MAX_IMPORTS 2048
#define TRADUCTOR_MAX_THREADS 16

// Cambiado a devolver el indice asignado (para que el loader pueda pisar el
// delay-slot del stub con el SYSCALL que codifica ese indice).
u32 traductor_registrarImport(u32 indice, u32 nid, const char *nombreLibreria);

// ============================================================
// Modulos PSP adicionales (ej: kjfs.prx)
// ============================================================
// El loader (main.cpp) parsea el PRX, aplica relocs y registra sus
// imports (via traductor_registrarImport) y sus exports (via
// traductor_registrarExport). Cuando el EBOOT (u otro modulo) llama un
// import cuyo NID coincide con un export registrado, el interprete
// "secuestra" el salto y ejecuta de verdad el codigo del modulo (igual
// que venimos haciendo con el EBOOT), haciendo el swap de $gp que
// exige cruzar de modulo.
//
// PC_MODULO_RET es un "return sentinel": las funciones de modulo devuelven
// a esta direccion ficticia, y el bucle del interprete la usa para restaurar
// el $gp del llamador y volver a quien llamo.
#define PC_MODULO_RET 0xFFFFFFFC

// Registra un modulo cargado. realMin/realMax = rango REAL (post-reloc) que
// ocupa el codigo/datos del modulo. vaddrMin/vaddrMax = rango VIRTUAL original
// (antes de relocs). gpValue = $gp real del modulo.
// moduleStart = direccion REAL de module_start (0 si no tiene). Devuelve el
// indice del modulo (>=0) o -1 si no hay lugar.
int  traductor_registrarModulo(u32 realMin, u32 realMax, u32 vaddrMin, u32 vaddrMax, u32 gpValue, u32 moduleStart, int tieneModuleStart);

// Registra un export de un modulo: nid -> addrReal (dentro del bloque del modulo).
void traductor_registrarExport(u32 nid, u32 addrReal, int moduloIdx);

// Ejecuta el module_start del modulo (si lo tiene). Debe llamarse DESPUES de
// registrar el modulo y ANTES de traductor_ejecutar (o cuando el juego lo pidiera).
void traductor_iniciarModulo(int idx);

// ============================================================
// Variables globales expuestas para nids_kjfs.c
// ============================================================
// EstadoHilo enum (shared)
typedef enum {
    HILO_LIBRE = 0,
    HILO_CREADO,
    HILO_LISTO,
    HILO_DURMIENDO,
    HILO_ESPERANDO_SEMA,
    HILO_ESPERANDO_PIPE,
    HILO_TERMINADO
} EstadoHilo;

// Hilo struct definition (shared with nids_kjfs.c)
// CpuSimulada is kept separate in traductor.c (parallel array)
typedef struct {
    u32 uid;
    u32 entry;
    u32 gp;
    u32 stackBase;
    u32 stackSize;
    EstadoHilo estado;
    u32 semaEsperado;
    s32 semaCantidad;
    u32 pipeEsperado;
    u32 wakeupsPendientes;
} Hilo;

extern u64 g_fakeTick;
extern u32 g_importNid[TRADUCTOR_MAX_IMPORTS];
extern const char *g_importNombreLib[TRADUCTOR_MAX_IMPORTS];
extern u32 g_cantidadImportsRegistrados;
extern int g_hiloActual;
extern Hilo g_hilos[TRADUCTOR_MAX_THREADS];

// Archivo abierto - expuesto para nids_kjfs.c
#define TRADUCTOR_MAX_ARCHIVOS 32
#define TRADUCTOR_DIR_MAXLEN 200

typedef struct {
    int usado;
    int esDir;
    FILE *f;
    DIR *d;
    char rutaDir[256];
} ArchivoAbierto;

extern char g_directorioDatos[TRADUCTOR_DIR_MAXLEN];
extern ArchivoAbierto g_archivosAbiertos[TRADUCTOR_MAX_ARCHIVOS];
extern void traducirRutaPSP(const char *rutaPsp, char *salida, size_t salidaMax);

// Funciones internas expuestas para delegación desde nids_kjfs.c
extern u8 *resolverDireccion(u32 addr);
extern u32 resolverImport(u32 indice, u32 a0, u32 a1, u32 a2, u32 a3, int *implementado);

#ifdef __cplusplus
}
#endif

#endif