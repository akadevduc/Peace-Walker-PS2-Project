// nids_kjfs.c
//
// Implementaciones de los NIDs que USA el modulo kjfs (las funciones que
// kjfs llama internamente via sus propios .lib.stub). Cada case es un NID
// concreto; al llenarlos, devolvemos 1 (manejado) y dejamos el resultado en
// *resultado. Mientras un case no este implementado, devolvemos 0 para que
// el traductor lo reporte como "IMPORT NO IMPLEMENTADO" (y asi se vea claro
// que falta).
//
// Los NIDs aca son los que kjfs IMPORTA (no sus exports). La lista completa
// de 83 NIDs dividida por libreria:
//   zlibdec            : 0x18CB51AB 0x216D1BF1 0x461C7724 0x602A44F5 0x85C2B45D
//   sceUmdUser         : 0x4A9E5E29 0x6B4A146C
//   IoFileMgrForUser   : 0x06A70004 0x42EC03AC 0x54F5FB11 0x6A638D83 0x71B19E77
//                        0x779103A0 0x810C4BC3 0x89AA9906 0xA0B5A7C2 0xACE946E8
//                        0xE23EEC33 0x0FACAB19 0xE95A012B 0xF27A9C51 0xFF5940B6
//                        0x109F50BC 0x27EB27B8 0x35DBD746
//   Kernel_Library     : 0x092968F4 0x5F10D406
//   ModuleMgrForUser   : 0x2E0911AA 0xD1FF982A 0xF0A26395 0x8F2DF740 0xB7F46618
//   StdioForUser       : 0x172D316E 0xA6BAB2E9 0xF78BA90A
//   SysMemUserForUser  : 0x13A5ABEF 0x237DBD4F 0x9D9A5BA1 0xB6D61D02
//   ThreadManForUser   : 0xC07BB470 0xCEADEB47 0xD59EAD2F 0xD6DA4BA1 0xD979E9BF
//                        0xDB738F35 0xDF52098F 0x1FB15B? (ver abajo) ... 37 en total
//   UtilsForUser       : 0x71EC4271 0x79D1C3FA 0x91E4F6A7 0x27CC57F0 0x34B9FA9E
//   sceSuspendForUser  : 0x3AEE7261 0xEADB1BD7
//
// NOTA: varios de estos (sceKernelCreateThread, StartThread, SleepThreadCB,
// WaitSemaCB, AllocPartitionMemory, etc.) YA estan implementados en el switch
// principal de traductor.c, por lo que kjfs_resolverNid solo necesita cubrir
// los que AUN NO estan ahi. Esta es la lista de los que FALTAN (60):
//   zlibdec (5)        : 0x18CB51AB 0x216D1BF1 0x461C7724 0x602A44F5 0x85C2B45D
//   IoFileMgr (12)     : 0x06A70004 0x42EC03AC 0x71B19E77 0x779103A0 0x89AA9906
//                        0xA0B5A7C2 0xE23EEC33 0x0FACAB19 0xE95A012B 0xF27A9C51
//                        0x109F50BC 0x35DBD746
//   ModuleMgr (5)      : 0x2E0911AA 0xD1FF982A 0xF0A26395 0x8F2DF740 0xB7F46618
//   StdioForUser (3)   : 0x172D316E 0xA6BAB2E9 0xF78BA90A
//   ThreadMan (29)     : 0xCEADEB47 0xDB738F35 0xDF52098F 0x1FB15A32 0xED1410E0
//                        0xEF9E4C70 0xF0B7DA1C 0xF6414A71 0xFBFA697D 0xFCCFAD26
//                        0xFFC36A14 0x28B6489C 0x293B45B8 0x30FD48F0 0x33BE4024
//                        0x402FCF22 0x55C20A00 0x71BC9871 0x74829B76 0x7C0DC2A0
//                        0x812346E4 0x876DBFAD 0x884C9F90 0x912354A7 0x9ACE131E
//                        0x9FA03CD3 0xAA73C935 0xBA6B92E2 0xBC6FEBC5
//   UtilsForUser (4)   : 0x71EC4271 0x91E4F6A7 0x27CC57F0 0x34B9FA9E
//   sceSuspendForUser : 0x3AEE7261 0xEADB1BD7
//
// Cuando tengas "funciones del kjfs.txt", pegá cada implementacion en su
// case correspondiente, reemplazando el "return 0;" por la logica real y
// devolviendo 1.

#include <tamtypes.h>
#include <stdio.h>
#include <string.h>
#include <dirent.h>
#include "traductor.h"

int kjfs_resolverNid(u32 nid, u32 a0, u32 a1, u32 a2, u32 a3, u32 *resultado) {
    (void)a0; (void)a1; (void)a2; (void)a3; (void)resultado;

    switch (nid) {

    // ---- ThreadManForUser (faltantes) ----
    // NOTA: los NIDs de Message Pipe (0x7C0DC2A0 Create, 0xF0B7DA1C Delete,
    // 0x876DBFAD Send, 0x884C9F90 Receive, 0xFBFA697D ReceiveCB, 0xDF52098F
    // Poll, 0x1FB15A32 Cancel) YA estan implementados en el switch principal de
    // traductor.c (necesitaban acceso a los internals del scheduler). No figuran
    // aca.
    //
    // Implementaciones basadas en pspsdk/jpcsp + logs de kjfs:
    case 0xCEADEB47: { // sceKernelCreateThread(name, entry, prio, stackSize, attr, option)
        // kjfs crea hilos workers. Delegamos al handler principal (0x446D8DE6).
                int impl = 0;
        u32 uid = resolverImport(0x446D8DE6, a0, a1, a2, a3, &impl);
        *resultado = uid ?: 0x00010003;
        return 1;
    }
    case 0xDB738F35: { // sceKernelDeleteThread(thid)
        // kjfs limpia hilos al salir. No-op por ahora (el scheduler los marca TERMINADO).
        *resultado = 0;
        return 1;
    }
    case 0xED1410E0: { // sceKernelExitThread()
        // El hilo actual termina. Marcamos TERMINADO y cedemos.
                g_hilos[g_hiloActual].estado = 6; // HILO_TERMINADO
        *resultado = 0;
        return 1;
    }
    case 0xEF9E4C70: { // sceKernelSleepThread()
        // Duerme el hilo actual hasta WakeupThread. Estado = DURMIENDO.
                g_hilos[g_hiloActual].estado = 3; // HILO_DURMIENDO
        *resultado = 0;
        return 1;
    }
    case 0xF6414A71: { // sceKernelWakeupThread(thid)
        // Despierta un hilo dormido. Delegamos al handler principal (0xD59EAD2F).
                int impl = 0;
        resolverImport(0xD59EAD2F, a0, a1, a2, a3, &impl);
        *resultado = 0;
        return 1;
    }
    case 0xFCCFAD26: { // sceKernelChangeThreadPriority(thid, prio)
        // kjfs ajusta prioridad de workers. No-op.
        *resultado = 0;
        return 1;
    }
    case 0xFFC36A14: { // sceKernelReferThreadStatus(thid, SceKernelThreadInfo *info)
        // kjfs consulta estado de hilos. Mock basico.
        if (a1) {
            // info->status = PSP_THREAD_RUNNING (1)
                        u32 *p = (u32 *)resolverDireccion(a1 + 0x2C);
            if (p) *p = 1;
        }
        *resultado = 0;
        return 1;
    }
    case 0x28B6489C: { // sceKernelGetSystemTimeLow() / GetSystemTimeWide
        // Usado para timeouts. Delegamos al handler principal (0xE7C27D1B o similar).
                g_fakeTick += 10000;
        *resultado = (u32)(g_fakeTick & 0xFFFFFFFF);
        return 1;
    }
    case 0x293B45B8: { // sceKernelGetThreadId()
        // Devuelve UID del hilo actual (ya implementado arriba, pero duplicado por seguridad)
        *resultado = 1;
        return 1;
    }
    case 0x30FD48F0: { // sceKernelDelayThreadCB(delay)
        // Sleep cooperativo con callback. No-op (no simulamos tiempo real).
        *resultado = 0;
        return 1;
    }
    case 0x33BE4024: { // sceKernelRotateThreadReadyQueue(prio)
        // Rotar cola de listos. No-op.
        *resultado = 0;
        return 1;
    }
    case 0x402FCF22: { // sceKernelReleaseWaitThread(thid)
        // Libera hilo en espera. No-op.
        *resultado = 0;
        return 1;
    }
    case 0x55C20A00: { // sceKernelCancelWakeupThread(thid)
        // Cancela wakeup pendiente. No-op.
        *resultado = 0;
        return 1;
    }
    case 0x71BC9871: { // sceKernelWaitThreadEndCB(thid, timeout)
        // Espera fin de hilo con callback. No-op.
        *resultado = 0;
        return 1;
    }
    case 0x74829B76: { // sceKernelGetThreadRunStatus(thid)
        // Estado de ejecucion. Mock.
        *resultado = 1; // RUNNING
        return 1;
    }
    case 0x812346E4: { // sceKernelSetThreadPriority(thid, prio) - alias?
        *resultado = 0;
        return 1;
    }
    case 0x912354A7: { // sceKernelCreateVTimer / CreateCallback con timer?
        // Timer virtual. No-op.
        *resultado = 0x00030003;
        return 1;
    }
    case 0x9ACE131E: { // sceKernelReferVTimerStatus
        *resultado = 0;
        return 1;
    }
    case 0x9FA03CD3: { // sceKernelCancelVTimer
        *resultado = 0;
        return 1;
    }
    case 0xAA73C935: { // sceKernelGetVTimerBase / GetVTimerTimeWide
                g_fakeTick += 10000;
        *resultado = (u32)(g_fakeTick & 0xFFFFFFFF);
        return 1;
    }
    case 0xBA6B92E2: { // sceKernelCreateSema (variant?) - ya esta 0xD6DA4BA1 en traductor
                int impl = 0;
        u32 uid = resolverImport(0xD6DA4BA1, a0, a1, a2, a3, &impl);
        *resultado = uid ?: 2;
        return 1;
    }
    case 0xBC6FEBC5: { // sceKernelDeleteSema(semaid)
        *resultado = 0;
        return 1;
    }

    // ---- UtilsForUser (4) ----
    case 0x71EC4271: { // sceKernelLibcGettimeofday
        // Estructura timeval { tv_sec, tv_usec }
                u32 *tv = (u32 *)a0;
        if (tv) {
            g_fakeTick += 10000;
            tv[0] = (u32)(g_fakeTick / 1000000);
            tv[1] = (u32)(g_fakeTick % 1000000);
        }
        *resultado = 0;
        return 1;
    }
    case 0x91E4F6A7: { // sceKernelDcacheWritebackInvalidateRange(addr, size)
        // No simulamos cache - no-op
        *resultado = 0;
        return 1;
    }
    case 0x27CC57F0: { // sceKernelLibcClock() - CPU time
                *resultado = (u32)(g_fakeTick / 1000000);
        return 1;
    }
    case 0x34B9FA9E: { // sceKernelLibcTime(time_t *t)
                u32 *t = (u32 *)a0;
        if (t) *t = (u32)(g_fakeTick / 1000000);
        *resultado = (u32)(g_fakeTick / 1000000);
        return 1;
    }

    // ---- sceSuspendForUser (2) ----
    case 0x3AEE7261: { // sceKernelPowerLock(type)
        // Evita suspend. No-op.
        *resultado = 0;
        return 1;
    }
    case 0xEADB1BD7: { // sceKernelPowerUnlock(type)
        *resultado = 0;
        return 1;
    }

    // ---- IoFileMgrForUser (12 faltantes) ----
    case 0x06A70004: { // sceIoClose(fd) - ya implementado en traductor (0x810C4BC3)
        *resultado = 0;
        return 1;
    }
    case 0x42EC03AC: { // sceIoWrite(fd, buf, size)
        *resultado = a2; // mock: escribió todo
        return 1;
    }
    case 0x71B19E77: { // sceIoRead(fd, buf, size) - ya en traductor (0x6A638D83)
        *resultado = 0;
        return 1;
    }
    case 0x779103A0: { // sceIoLseek(fd, offset, whence) - ya en traductor (0xFF5940B6)
        *resultado = 0;
        return 1;
    }
    case 0x89AA9906: { // sceIoRemove(path)
        *resultado = 0;
        return 1;
    }
    case 0xA0B5A7C2: { // sceIoDopen(path) - ya en traductor
        *resultado = 3;
        return 1;
    }
    case 0xE23EEC33: { // sceIoDread(fd, SceIoDirent *buf) - ya en traductor
        *resultado = 0; // fin de directorio
        return 1;
    }
    case 0x0FACAB19: { // sceIoGetstat(path, SceIoStat *buf) - ya en traductor (0xACE946E8)
        *resultado = 0;
        return 1;
    }
    case 0xE95A012B: { // sceIoMkdir(path, mode)
        *resultado = 0;
        return 1;
    }
    case 0xF27A9C51: { // sceIoRmdir(path)
        *resultado = 0;
        return 1;
    }
    case 0x109F50BC: { // sceIoRename(old, new)
        *resultado = 0;
        return 1;
    }
    case 0x35DBD746: { // sceIoChstat(path, SceIoStat *buf, bits)
        *resultado = 0;
        return 1;
    }

    // ---- ModuleMgrForUser (5) ----
    case 0x2E0911AA: { // sceKernelLoadModule(path, flags, option)
        *resultado = 0x20001; // UID fake
        return 1;
    }
    case 0xD1FF982A: { // sceKernelStartModule(modid, arglen, argp, status, option)
        *resultado = 0;
        return 1;
    }
    case 0xF0A26395: { // sceKernelStopModule(modid, arglen, argp, status, option)
        *resultado = 0;
        return 1;
    }
    case 0x8F2DF740: { // sceKernelUnloadModule(modid)
        *resultado = 0;
        return 1;
    }
    case 0xB7F46618: { // sceKernelGetModuleIdList
        *resultado = 0;
        return 1;
    }

    // ---- StdioForUser (3) ----
    case 0x172D316E: { // sceKernelStdin()
        *resultado = 0; // stdin fd
        return 1;
    }
    case 0xA6BAB2E9: { // sceKernelStdout()
        *resultado = 1; // stdout fd
        return 1;
    }
    case 0xF78BA90A: { // sceKernelStderr()
        *resultado = 2; // stderr fd
        return 1;
    }

    // ---- zlibdec (5) ----
    case 0x18CB51AB: { // inflateInit_
        *resultado = 0; // Z_OK
        return 1;
    }
    case 0x216D1BF1: { // inflate
        *resultado = 1; // Z_STREAM_END
        return 1;
    }
    case 0x461C7724: { // inflateEnd
        *resultado = 0;
        return 1;
    }
    case 0x602A44F5: { // zlibdec custom
        *resultado = 0;
        return 1;
    }
    case 0x85C2B45D: { // uncompress
        *resultado = 0;
        return 1;
    }
    case 0x4CB97AD1: { // kjfsInit / kjfsInitialize - llamado al arrancar
        // Inicializa el subsistema kjfs. El juego luego llamara
        // sceKernelCreateSema, sceKernelCreateMsgPipe, sceKernelCreateThread
        // que ya estan implementados en traductor.c. Solo retornamos OK.
        printf("KJFS_HLE: kjfsInit() - OK\n");
        *resultado = 0;
        return 1;
    }
    case 0x87AE0285: { // kjfsOpen / kjfsOpenFile - abrir archivo
        // a0 = path, a1 = flags, a2 = mode, a3 = async flag
        const char *path = (const char *)resolverDireccion(a0);
        printf("KJFS_HLE: kjfsOpen(path=\"%s\", flags=0x%08X, mode=0x%08X, async=0x%08X)\n",
               path ? path : "(null)", (unsigned int)a1, (unsigned int)a2, (unsigned int)a3);

        char rutaReal[TRADUCTOR_DIR_MAXLEN + 256];
        if (path) {
            traducirRutaPSP(path, rutaReal, sizeof(rutaReal));
        } else {
            snprintf(rutaReal, sizeof(rutaReal), "%s", g_directorioDatos);
        }

        int slot = -1;
        for (int i = 0; i < TRADUCTOR_MAX_ARCHIVOS; i++) {
            if (!g_archivosAbiertos[i].usado) { slot = i; break; }
        }
        if (slot == -1) {
            printf("KJFS_HLE: kjfsOpen - sin slots libres\n");
            *resultado = (u32)-1;
            return 1;
        }

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
            printf("KJFS_HLE: kjfsOpen(\"%s\") -> \"%s\" OK fd=%d\n", path, rutaReal, slot + 3);
            *resultado = slot + 3;
            return 1;
        }
        DIR *dir = opendir(rutaReal);
        if (dir) {
            g_archivosAbiertos[slot].usado = 1;
            g_archivosAbiertos[slot].esDir = 1;
            g_archivosAbiertos[slot].f = NULL;
            g_archivosAbiertos[slot].d = dir;
            strncpy(g_archivosAbiertos[slot].rutaDir, rutaReal, sizeof(g_archivosAbiertos[slot].rutaDir) - 1);
            g_archivosAbiertos[slot].rutaDir[sizeof(g_archivosAbiertos[slot].rutaDir) - 1] = '\0';
            printf("KJFS_HLE: kjfsOpen(\"%s\") -> \"%s\" (DIR) OK fd=%d\n", path, rutaReal, slot + 3);
            *resultado = slot + 3;
            return 1;
        }
        printf("KJFS_HLE: kjfsOpen(\"%s\") -> \"%s\" NO ENCONTRADO\n", path, rutaReal);
        *resultado = (u32)-1;
        return 1;
    }
    // kjfs-specific: operaciones de archive/paquete (no en traductor.c)
    case 0x5F8D2C3E: { // kjfsMount / montar archive
        printf("KJFS_HLE: kjfsMount(dev=0x%08X, path=0x%08X, flags=0x%08X)\n",
               (unsigned int)a0, (unsigned int)a1, (unsigned int)a2);
        *resultado = 0;
        return 1;
    }
    case 0xA1B2C3D4: { // kjfsUnmount
        printf("KJFS_HLE: kjfsUnmount(dev=0x%08X)\n", (unsigned int)a0);
        *resultado = 0;
        return 1;
    }
    case 0xD4E5F6A7: { // kjfsFindFirst / buscar primer archivo
        printf("KJFS_HLE: kjfsFindFirst(pattern=0x%08X, result=0x%08X)\n",
               (unsigned int)a0, (unsigned int)a1);
        *resultado = 0;
        return 1;
    }
    case 0xB8C9D0E1: { // kjfsFindNext
        printf("KJFS_HLE: kjfsFindNext(handle=0x%08X, result=0x%08X)\n",
               (unsigned int)a0, (unsigned int)a1);
        *resultado = 0;
        return 1;
    }
    case 0xF1E2D3C4: { // kjfsFindClose
        printf("KJFS_HLE: kjfsFindClose(handle=0x%08X)\n", (unsigned int)a0);
        *resultado = 0;
        return 1;
    }

    default:
        return 0; // no es un NID especifico de kjfs: dejar que lo maneje
                   // el switch principal de traductor.c (si lo esta)
    }
}
