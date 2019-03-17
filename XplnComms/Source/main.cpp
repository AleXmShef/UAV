#define XPLM301
#define XPLM300
#define XPLM210
#define XPLM200
#include "XPLMDisplay.h"
#include "XPLMProcessing.h"
#include "XPLMGraphics.h"
#include "XPLMMenus.h"
#include "Guidance.h"
#include <string.h>
#include "stdio.h"
//#include <boost/thread/thread.hpp>
#include <thread>

#include "Guidance.h"
#if IBM
    #include <windows.h>
    #include <GL/gl.h>
    BOOL APIENTRY DllMain( HANDLE hModule,
                           DWORD ul_reason_for_call,
                           LPVOID lpReserved
    )
    {
        switch (ul_reason_for_call)
        {
            case DLL_PROCESS_ATTACH:
            case DLL_THREAD_ATTACH:
            case DLL_THREAD_DETACH:
            case DLL_PROCESS_DETACH:
                break;
        }
        return TRUE;
    }
#endif
#if LIN
#include <GL/gl.h>
#elif __GNUC__

#else
#include <GL/gl.h>
#endif

#ifndef XPLM301
#error This is made to be compiled against the XPLM301 SDK
#endif

// An opaque handle to the window we will create
static XPLMWindowID	g_window;

// Callbacks we will register when we create our window
void				draw_hello_world(XPLMWindowID in_window_id, void * in_refcon);
int					dummy_mouse_handler(XPLMWindowID in_window_id, int x, int y, int is_down, void * in_refcon) { return 0; }
XPLMCursorStatus	dummy_cursor_status_handler(XPLMWindowID in_window_id, int x, int y, void * in_refcon) { return xplm_CursorDefault; }
int					dummy_wheel_handler(XPLMWindowID in_window_id, int x, int y, int wheel, int clicks, void * in_refcon) { return 0; }
void				dummy_key_handler(XPLMWindowID in_window_id, char key, XPLMKeyFlags flags, char virtual_key, void * in_refcon, int losing_focus) { }
float DataUpdateCallback(float inElapsedSinceLastCall, float inElapsedSinceFlightLoop, int inCounter, void* inRefcon);
void MyMenuCallback(void* inMenuRef, void* inItemRef);
void* PFDMenuItem;
void* SwitchControlsMenuItem;

PLUGIN_API int XPluginStart(
        char *		outName,
        char *		outSig,
        char *		outDesc)
{
    strcpy(outName, "HelloWorld3Plugin");
    strcpy(outSig, "xpsdk.examples.helloworld3plugin");
    strcpy(outDesc, "A Hello World plug-in for the XPLM300 SDK.");

    XPLMCreateWindow_t params;
    params.structSize = sizeof(params);
    params.visible = 0;
    params.drawWindowFunc = draw_hello_world;
    // Note on "dummy" handlers:
    // Even if we don't want to handle these events, we have to register a "do-nothing" callback for them
    params.handleMouseClickFunc = dummy_mouse_handler;
    params.handleRightClickFunc = dummy_mouse_handler;
    params.handleMouseWheelFunc = dummy_wheel_handler;
    params.handleKeyFunc = dummy_key_handler;
    params.handleCursorFunc = dummy_cursor_status_handler;
    params.refcon = NULL;
    params.layer = xplm_WindowLayerFloatingWindows;
    // Opt-in to styling our window like an X-Plane 11 native window
    // If you're on XPLM300, not XPLM301, swap this enum for the literal value 1.
    params.decorateAsFloatingWindow = xplm_WindowDecorationRoundRectangle;

    // Set the window's initial bounds
    // Note that we're not guaranteed that the main monitor's lower left is at (0, 0)...
    // We'll need to query for the global desktop bounds!
    int left, bottom, right, top;
    XPLMGetScreenBoundsGlobal(&left, &top, &right, &bottom);
    params.left = left + 50;
    params.bottom = bottom + 150;
    params.right = params.left + 250;
    params.top = params.bottom + 250;

    g_window = XPLMCreateWindowEx(&params);

    // Position the window as a "free" floating window, which the user can drag around
    XPLMSetWindowPositioningMode(g_window, xplm_WindowPositionFree, -1);
    // Limit resizing our window: maintain a minimum width/height of 100 boxels and a max width/height of 300 boxels
    XPLMSetWindowResizingLimits(g_window, 200, 200, 500, 500);
    XPLMSetWindowTitle(g_window, "Sample Window");

    int my_slot = XPLMAppendMenuItem(XPLMFindPluginsMenu(), "PFD Monitor", NULL, 0);
    XPLMMenuID m = XPLMCreateMenu("PFD Monitor", XPLMFindPluginsMenu(), my_slot, MyMenuCallback, NULL);
    XPLMAppendMenuItem(m, "Show PFD", (void*)1, 0);
    XPLMAppendMenuItem(m, "Switch Controls", (void*)2, 0);

    XCOM::Guidance::GetInstance();
    XPLMRegisterFlightLoopCallback(DataUpdateCallback, -1.0, NULL);

    //register menu
    //register window for menu
    //initialize guidance module
    //button for menu
    //button callback

    return g_window != NULL;
}

PLUGIN_API void	XPluginStop(void)
{
    // Since we created the window, we'll be good citizens and clean it up
    XPLMDestroyWindow(g_window);
    g_window = NULL;
}

PLUGIN_API void XPluginDisable(void) { }
PLUGIN_API int  XPluginEnable(void)  { return 1; }
PLUGIN_API void XPluginReceiveMessage(XPLMPluginID inFrom, int inMsg, void * inParam) { }

static void multMatrixVec4f(GLfloat dst[4], const GLfloat m[16], const GLfloat v[4])
{
    dst[0] = v[0] * m[0] + v[1] * m[4] + v[2] * m[8] + v[3] * m[12];
    dst[1] = v[0] * m[1] + v[1] * m[5] + v[2] * m[9] + v[3] * m[13];
    dst[2] = v[0] * m[2] + v[1] * m[6] + v[2] * m[10] + v[3] * m[14];
    dst[3] = v[0] * m[3] + v[1] * m[7] + v[2] * m[11] + v[3] * m[15];
}

static void modelview_to_window_coords(int out_w[2], const GLfloat in_mv[4], const GLfloat mv[16], const GLfloat pr[16], const GLint viewport[4])
{
    GLfloat eye[4], ndc[4];
    multMatrixVec4f(eye, mv, in_mv);
    multMatrixVec4f(ndc, pr, eye);
    ndc[3] = 1.0f / ndc[3];
    ndc[0] *= ndc[3];
    ndc[1] *= ndc[3];

    out_w[0] = (ndc[0] * 0.5f + 0.5f) * viewport[2] + viewport[0];
    out_w[1] = (ndc[1] * 0.5f + 0.5f) * viewport[3] + viewport[1];
}

void draw_hello_world(XPLMWindowID in_window_id, void * in_refcon)
{
    // Mandatory: We *must* set the OpenGL state before drawing
    // (we can't make any assumptions about it)



    XPLMSetGraphicsState(
            0 /* no fog */,
            0 /* 0 texture units */,
            0 /* no lighting */,
            0 /* no alpha testing */,
            1 /* do alpha blend */,
            1 /* do depth testing */,
            0 /* no depth writing */
    );

    int l, t, r, b;
    XPLMGetWindowGeometry(in_window_id, &l, &t, &r, &b);

    static XPLMDataRef mv_dref = XPLMFindDataRef("sim/graphics/view/modelview_matrix");
    static XPLMDataRef vp_dref = XPLMFindDataRef("sim/graphics/view/viewport");
    static XPLMDataRef pr_dref = XPLMFindDataRef("sim/graphics/view/projection_matrix");

    // Get the current modelview matrix, viewport, and projection matrix from X-Plane
    float mv[16], pr[16];
    int vp[4];
    XPLMGetDatavf(mv_dref, mv, 0, 16);
    XPLMGetDatavf(pr_dref, pr, 0, 16);
    XPLMGetDatavi(vp_dref, vp, 0, 4);

    // Our new modelview bounds: we'll bring the window in by 10 bx on all sides
    GLfloat top_right_modelview[4] = { (float)r - 10, (float)t - 10, 0, 1 };
    GLfloat btm_left_modelview[4] =  { (float)l + 10, (float)b + 10, 0, 1 };

    // Get our top-right and bottom-left window coordinates
    int top_right_window[2], btm_left_window[2];
    modelview_to_window_coords(top_right_window, top_right_modelview, mv, pr, vp);
    modelview_to_window_coords(btm_left_window,  btm_left_modelview,  mv, pr, vp);


    float sc = (r - l)/200.0;

    //Speed rect
    float srtl[] = {6*sc, 180*sc};
    float srbl[] = {6*sc, 20*sc};
    float srtr[] = {33*sc, 180*sc};
    float srbr[] = {33*sc, 20*sc};

    //Alt rect
    float artl[] = {167*sc, 180*sc};
    float arbl[] = {167*sc, 20*sc};
    float artr[] = {195*sc, 180*sc};
    float arbr[] = {195*sc, 20*sc};

    //PFD Main rect
    float mrtl[] = {46*sc, 157*sc};
    float mrbl[] = {46*sc, 48*sc};
    float mrtr[] = {152*sc, 157*sc};
    float mrbr[] = {152*sc, 48*sc};

    //PFD Att cross
    //vertical
    float actl[] = {99*sc, 115*sc};
    float acbl[] = {99*sc, 85*sc};
    float actr[] = {101*sc, 115*sc};
    float acbr[] = {101*sc, 85*sc};
    //horizontal
    float ahtl[] = {85*sc, 101*sc};
    float ahbl[] = {85*sc, 99*sc};
    float ahtr[] = {115*sc, 101*sc};
    float ahbr[] = {115*sc, 99*sc};

    glFrontFace(GL_CW);

    //filling black square
//    glColor4f(0, 0, 0, 1);  //black
//    glBegin(GL_QUADS);
//    {
//        glVertex2f(l, b);
//        glVertex2f(l, t);
//        glVertex2f(r, t);
//        glVertex2f(r, b);
//    }
//    glEnd();

    //filling speed rect & alt rect
    glColor4f(154.0f/255.0f, 141.0f/255.0f, 141.0f/255.0f, 1);  //grey
    glBegin(GL_TRIANGLE_STRIP);
    {
        glVertex2f(l+srbl[0], b+srbl[1]);
        glVertex2f(l+srtl[0], b+srtl[1]);
        glVertex2f(l+srbr[0], b+srbr[1]);
        glVertex2f(l+srtr[0], b+srtr[1]);

    }
    glEnd();
    glBegin(GL_TRIANGLE_STRIP);
    {
        glVertex2f(l+arbl[0], b+arbl[1]);
        glVertex2f(l+artl[0], b+artl[1]);
        glVertex2f(l+arbr[0], b+arbr[1]);
        glVertex2f(l+artr[0], b+artr[1]);
    }
    glEnd();

    //filling main pfd rect
    glColor4f(0, 127.0f/255.0f, 1, 1);  //sky
    glBegin(GL_TRIANGLE_STRIP);
    {
        glVertex2f(l+mrbl[0], b+mrbl[1]);
        glVertex2f(l+mrtl[0], b+mrtl[1]);
        glVertex2f(l+mrbr[0], b+mrbr[1]);
        glVertex2f(l+mrtr[0], b+mrtr[1]);

    }
    glEnd();

    //Write speed in speed rect
    float col_white[] = {1.0, 1.0, 1.0}; // red, green, blue
    glEnable(GL_SCISSOR_TEST);
    glScissor(l + srbl[0], b+srbl[1], srbr[0] - srbl[0], srtr[1] - srbr[1]);
    {
        float spd = XPLMGetDataf(XPLMFindDataRef("sim/flightmodel/position/indicated_airspeed"));
        int h = (spd/10)*12;
        for (int i = 0; i < spd + 80; i+= 10) {
            char buff[3];
            sprintf(buff, "%d", i);
            XPLMDrawString(col_white, l + 8*sc, t - (100 + h)*sc, buff, NULL, xplmFont_Proportional);
            h-=12;
        }

    }
    glDisable(GL_SCISSOR_TEST);

    //write altitiude in alt rect
    glEnable(GL_SCISSOR_TEST);
    glScissor(l + arbl[0], b+arbl[1], arbr[0] - arbl[0], artr[1] - arbr[1]);
    {
        float alt = XPLMGetDataf(XPLMFindDataRef("sim/flightmodel/position/elevation"));
        alt*=3.281;
        int h = (alt/100)*18;
        for (int i = 0; i < alt + 500; i+= 100) {
            char buff[4];
            sprintf(buff, "%d", i);
            XPLMDrawString(col_white, r - 30*sc, t - (100 + h)*sc, buff, NULL, xplmFont_Proportional);
            h-=18;
        }

    }
    glDisable(GL_SCISSOR_TEST);


    //drawing ground
    float pitch = XPLMGetDataf(XPLMFindDataRef("sim/flightmodel/position/true_theta"));
    float roll = -1*XPLMGetDataf(XPLMFindDataRef("sim/flightmodel/position/true_phi"))*M_PI/180;

    float y = 100 - pitch;

    float a = 56/cos(roll);
    float c = a*sin(roll);
    float grtl[] = {44*sc, (c + 100 - pitch)*sc};
    float grbl[] = {44*sc, -25*sc};
    float grtr[] = {156*sc, (100-pitch-c)*sc};
    float grbr[] = {156*sc, -25*sc};

    glEnable(GL_SCISSOR_TEST);
    glScissor(l + mrbl[0], b+mrbl[1], mrbr[0] - mrbl[0], mrtr[1] - mrbr[1]);
    glColor4f(147.0f/255.0f, 35.0f/255.0f, 0, 1);  //black
    glBegin(GL_TRIANGLE_STRIP);
    {
        glVertex2f(l+grbl[0], b+grbl[1]);
        glVertex2f(l+grtl[0], b+grtl[1]);
        glVertex2f(l+grbr[0], b+grbr[1]);
        glVertex2f(l+grtr[0], b+grtr[1]);

    }
    glEnd();

    //drawing attitude indicator
    glColor4f(0, 0, 0, 1);  //black
    glBegin(GL_TRIANGLE_STRIP);
    {
        glVertex2f(l+acbl[0], b+acbl[1]);
        glVertex2f(l+actl[0], b+actl[1]);
        glVertex2f(l+acbr[0], b+acbr[1]);
        glVertex2f(l+actr[0], b+actr[1]);

    }
    glEnd();
    glBegin(GL_TRIANGLE_STRIP);
    {
        glVertex2f(l+ahbl[0], b+ahbl[1]);
        glVertex2f(l+ahtl[0], b+ahtl[1]);
        glVertex2f(l+ahbr[0], b+ahbr[1]);
        glVertex2f(l+ahtr[0], b+ahtr[1]);

    }
    glEnd();

    //Drawing flight directors











    //XPLMDrawString(col_white, l + 10, t - 20, "Hello World", NULL, xplmFont_Proportional);
}

void DataUpdateFunction() {
    XCOM::Guidance::GetInstance()->Update();
}

float DataUpdateCallback(float inElapsedSinceLastCall, float inElapsedSinceFlightLoop, int inCounter, void* inRefcon) {
//    XCOM::Guidance::GetInstance()->GetThread()->join();
//    XCOM::Guidance::GetInstance()->mThread = new std::thread(DataUpdateFunction);
    DataUpdateFunction();
    return -1.0;
}

void MyMenuCallback(void* inMenuRef, void* inItemRef) {
    if (inItemRef == (void*)1) {
        XPLMSetWindowIsVisible(g_window, 1);
    }
    else if (inItemRef == (void*)2) {
        XCOM::Guidance::GetInstance()->SwitchControls();
    }
}
