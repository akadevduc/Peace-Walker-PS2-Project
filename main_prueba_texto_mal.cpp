#define F_gsKit_texture_png

#include <tamtypes.h>
#include <kernel.h>
#include <gsKit.h>
#include <gsToolkit.h>
#include <dmaKit.h>
#include <graph.h>
#include <math.h>
#include <stdio.h>
#include <sifrpc.h>

extern "C" int gsKit_texture_png(GSGLOBAL *gsGlobal, GSTEXTURE *Texture, char *Path);

GSTEXTURE fontTexture;

struct Glyph
{
    int x;
    int y;
    int w;
    int h;
    int advance;
};

Glyph glyphs[26] =
{
    {348,0,36,32,24},{384,0,38,32,26},{422,0,38,32,26},{460,0,39,32,24},
    {0,44,34,32,22},{278,0,35,33,21},{0,0,37,44,26},{313,0,35,33,23},
    {34,44,22,32,10},{215,0,34,38,21},{56,44,41,32,27},{97,44,25,32,22},
    {139,0,45,43,31},{122,44,37,32,24},{159,44,33,32,21},{192,44,34,32,24},
    {226,44,31,32,30},{184,0,31,43,27},{257,44,36,32,21},{293,44,33,32,24},
    {326,44,35,32,21},{361,44,30,32,23},{391,44,46,32,34},{437,44,49,32,23},
    {0,76,28,32,17},{28,76,32,32,22}
};

static void draw_text(GSGLOBAL *gsGlobal, const char *text, float x, float y)
{
    while(*text)
    {
        char c=*text;

        if(c>='A' && c<='Z')
        {
            Glyph &g=glyphs[c-'A'];

            float u1=(float)g.x/(float)fontTexture.Width;
            float v1=(float)g.y/(float)fontTexture.Height;
            float u2=(float)(g.x+g.w)/(float)fontTexture.Width;
            float v2=(float)(g.y+g.h)/(float)fontTexture.Height;

            gsKit_prim_sprite_texture(
                gsGlobal,
                &fontTexture,
                x,
                y,
                u1,
                v1,
                x+g.w,
                y+g.h,
                u2,
                v2,
                0,
                GS_SETREG_RGBAQ(255,0,0,0x80,0)
            );

            x+=g.advance;
        }
        else if(c==' ')
        {
            x+=16;
        }

        text++;
    }
}

static void hue_to_rgb(float hue, u8 *r, u8 *g, u8 *b)
{
    float c = 1.0f;
    float x = c * (1.0f - fabsf(fmodf(hue / 60.0f, 2.0f) - 1.0f));
    float r1 = 0.0f;
    float g1 = 0.0f;
    float b1 = 0.0f;

    if(hue < 60.0f)
    {
        r1 = c;
        g1 = x;
    }
    else if(hue < 120.0f)
    {
        r1 = x;
        g1 = c;
    }
    else if(hue < 180.0f)
    {
        g1 = c;
        b1 = x;
    }
    else if(hue < 240.0f)
    {
        g1 = x;
        b1 = c;
    }
    else if(hue < 300.0f)
    {
        r1 = x;
        b1 = c;
    }
    else
    {
        r1 = c;
        b1 = x;
    }

    *r = (u8)(r1 * 255);
    *g = (u8)(g1 * 255);
    *b = (u8)(b1 * 255);
}

int main(int argc, char *argv[])
{
    SifInitRpc(0);

    dmaKit_init(
        D_CTRL_RELE_OFF,
        D_CTRL_MFD_OFF,
        D_CTRL_STS_UNSPEC,
        D_CTRL_STD_OFF,
        D_CTRL_RCYC_8,
        1 << DMA_CHANNEL_GIF
    );

    dmaKit_chan_init(DMA_CHANNEL_GIF);

    GSGLOBAL *gsGlobal = gsKit_init_global();

    gsGlobal->Mode = graph_get_region() == GRAPH_MODE_PAL ? GS_MODE_PAL : GS_MODE_NTSC;
    gsGlobal->Width = 640;
    gsGlobal->Height = (gsGlobal->Mode == GS_MODE_PAL) ? 512 : 448;
    gsGlobal->Interlace = GS_INTERLACED;
    gsGlobal->Field = GS_FIELD;
    gsGlobal->PSM = GS_PSM_CT24;
    gsGlobal->PSMZ = GS_PSMZ_16S;
    gsGlobal->DoubleBuffering = GS_SETTING_ON;
    gsGlobal->ZBuffering = GS_SETTING_OFF;
    gsGlobal->PrimAlphaEnable = GS_SETTING_OFF;

    gsKit_init_screen(gsGlobal);
    gsKit_mode_switch(gsGlobal, GS_ONESHOT);

    const char *prefijos[] = { "mass:", "host:" };
    int resultadoCarga = -1;

    for (int i = 0; i < 2 && resultadoCarga < 0; i++)
    {
        char texturePath[64];
        snprintf(texturePath, sizeof(texturePath), "%sfontMGS.png", prefijos[i]);

        resultadoCarga = gsKit_texture_png(gsGlobal, &fontTexture, texturePath);

        if (resultadoCarga >= 0)
        {
            printf("Textura cargada desde: %s\n", texturePath);
        }
    }

    printf("Resultado de carga: %d\n", resultadoCarga);

    if(resultadoCarga < 0)
    {
        printf("NO CARGO TEXTURA\n");
    }
    else
    {
        printf("TEXTURA OK %d %d\n",fontTexture.Width,fontTexture.Height);

        gsKit_texture_upload(gsGlobal,&fontTexture);

        printf("PSM de la textura: %d (CT32=%d con alfa, CT24=%d sin alfa)\n", fontTexture.PSM, GS_PSM_CT32, GS_PSM_CT24);

        fontTexture.Filter=GS_FILTER_NEAREST;
    }

    gsKit_texture_upload(gsGlobal, &fontTexture);

    const u64 gris = GS_SETREG_RGBAQ(125,125,125,0x80,0);
    float hue = 0.0f;

    while(1)
    {
        u8 r,g,b;
        hue_to_rgb(hue,&r,&g,&b);

        u64 color = GS_SETREG_RGBAQ(r,g,b,0x80,0);

        gsKit_clear(gsGlobal, gris);

        gsKit_prim_sprite(
            gsGlobal,
            200,
            150,
            440,
            300,
            0,
            color
        );
/*
        draw_text(
            gsGlobal,
            "HOLA PS2",
            200,
            330
        );
*/
        gsKit_queue_exec(gsGlobal);
        gsKit_sync_flip(gsGlobal);

        hue += 1.0f;
        if(hue >= 360.0f)
            hue -= 360.0f;
    }

    return 0;
}