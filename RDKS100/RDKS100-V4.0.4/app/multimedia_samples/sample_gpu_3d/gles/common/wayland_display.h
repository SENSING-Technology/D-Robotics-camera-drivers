#ifndef  __WAYLAND_DISPLAY__HH_
#define __WAYLAND_DISPLAY__HH_
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdbool.h>
#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <wayland-client.h>
#include <wayland-egl.h>
#include <xdg-shell-client-protocol.h>

typedef struct {
	int width;
	int height;

	struct wl_display *wl_display;
	struct wl_registry *wl_registry;
	struct wl_compositor *wl_compositor;
	struct wl_surface *wl_surface;
	struct wl_egl_window *wl_egl_window;
	struct xdg_wm_base *xdg_wm_base;
	struct xdg_surface *xdg_surface;
	struct xdg_toplevel *xdg_toplevel;

	EGLDisplay egl_display;
	EGLContext egl_context;
	EGLSurface egl_surface;

	bool running;
} WaylandDisplay;

WaylandDisplay* wayland_display_create(int width, int height);
int wayland_display_init(WaylandDisplay *display);
void wayland_display_deinit(WaylandDisplay *display);
void wayland_display_destroy(WaylandDisplay *display);
void wayland_display_make_current(WaylandDisplay *display);
bool wayland_display_is_running(WaylandDisplay *display);
void wayland_display_swap_buffers(WaylandDisplay *display);
#endif // ! __WAYLAND_DISPLAY__HH_
