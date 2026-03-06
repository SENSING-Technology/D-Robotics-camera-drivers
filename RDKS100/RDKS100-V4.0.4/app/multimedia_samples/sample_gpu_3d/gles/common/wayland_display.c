#include "wayland_display.h"

typedef EGLDisplay (*PFNEGLGETPLATFORMDISPLAYEXTPROC)(EGLenum platform, void *native_display, const EGLint *attrib_list);

/****************************************************************************
 * Listener Implementations
 ****************************************************************************/

static void registry_handle_global(void *data, struct wl_registry *registry,
								 uint32_t id, const char *interface, uint32_t version) {
	WaylandDisplay *display = (WaylandDisplay *)data;
	if (strcmp(interface, "wl_compositor") == 0) {
		display->wl_compositor = wl_registry_bind(registry, id, &wl_compositor_interface, 4);
	} else if (strcmp(interface, "xdg_wm_base") == 0) {
		display->xdg_wm_base = wl_registry_bind(registry, id, &xdg_wm_base_interface, 1);
	}
}

static void registry_handle_global_remove(void *data, struct wl_registry *registry,
										uint32_t id) {
	// No action needed
}

static void xdg_wm_base_ping(void *data, struct xdg_wm_base *base, uint32_t serial) {
	xdg_wm_base_pong(base, serial);
}

static void xdg_surface_handle_configure(void *data, struct xdg_surface *surface, uint32_t serial) {
	xdg_surface_ack_configure(surface, serial);
}

static void xdg_toplevel_handle_configure(void *data, struct xdg_toplevel *toplevel,
										int32_t width, int32_t height,
										struct wl_array *states) {
	// Handle resize if needed
}

static void xdg_toplevel_handle_close(void *data, struct xdg_toplevel *toplevel) {
	WaylandDisplay *display = (WaylandDisplay *)data;
	display->running = false;
}

/****************************************************************************
 * Listener Structures
 ****************************************************************************/

static const struct wl_registry_listener registry_listener = {
	registry_handle_global,
	registry_handle_global_remove
};

static const struct xdg_wm_base_listener wm_base_listener = {
	xdg_wm_base_ping
};

static const struct xdg_surface_listener surface_listener = {
	xdg_surface_handle_configure
};

static const struct xdg_toplevel_listener toplevel_listener = {
	xdg_toplevel_handle_configure,
	xdg_toplevel_handle_close
};

/****************************************************************************
 * WaylandDisplay Functions
 ****************************************************************************/

WaylandDisplay* wayland_display_create(int width, int height) {
	WaylandDisplay *display = malloc(sizeof(WaylandDisplay));
	if (!display) return NULL;

	memset(display, 0, sizeof(WaylandDisplay));
	display->width = width;
	display->height = height;
	display->running = true;

	display->wl_display = NULL;
	display->wl_registry = NULL;
	display->wl_compositor = NULL;
	display->wl_surface = NULL;
	display->wl_egl_window = NULL;
	display->xdg_wm_base = NULL;
	display->xdg_surface = NULL;
	display->xdg_toplevel = NULL;

	display->egl_display = EGL_NO_DISPLAY;
	display->egl_context = EGL_NO_CONTEXT;
	display->egl_surface = EGL_NO_SURFACE;

	return display;
}

int wayland_display_init(WaylandDisplay *display) {
	// 1. Connect to the Wayland display server
	const char *wayland_display = getenv("WAYLAND_DISPLAY");
	char socket_path[256];

	if (wayland_display) {
		printf("Using WAYLAND_DISPLAY: %s\n", wayland_display);
		display->wl_display = wl_display_connect(wayland_display);
	} else {
		// Get current user UID
		uid_t uid = geteuid();
		snprintf(socket_path, sizeof(socket_path), "/run/user/%d/wayland-0", uid);

		struct stat st;
		if (stat(socket_path, &st) == 0 && S_ISSOCK(st.st_mode)) {
			printf("Using default socket path: %s\n", socket_path);
			display->wl_display = wl_display_connect(socket_path);
		} else {
			fprintf(stderr, "Wayland socket not found: %s\n", socket_path);
			return -1;
		}
	}

	if (!display->wl_display) {
		fprintf(stderr, "Failed to connect to Wayland display.\n");
		return -1;
	}

	// 2. Get and listen to the Wayland registry
	display->wl_registry = wl_display_get_registry(display->wl_display);
	wl_registry_add_listener(display->wl_registry, &registry_listener, display);
	wl_display_roundtrip(display->wl_display);

	if (!display->wl_compositor || !display->xdg_wm_base) {
		fprintf(stderr, "Failed to get wl_compositor or xdg_wm_base.\n");
		return -1;
	}

	// 3. Add xdg_wm_base listener
	xdg_wm_base_add_listener(display->xdg_wm_base, &wm_base_listener, NULL);

	// 4. Create Wayland surface and xdg_surface and xdg_toplevel
	display->wl_surface = wl_compositor_create_surface(display->wl_compositor);
	display->xdg_surface = xdg_wm_base_get_xdg_surface(display->xdg_wm_base, display->wl_surface);
	xdg_surface_add_listener(display->xdg_surface, &surface_listener, NULL);

	display->xdg_toplevel = xdg_surface_get_toplevel(display->xdg_surface);
	xdg_toplevel_add_listener(display->xdg_toplevel, &toplevel_listener, display);
	xdg_toplevel_set_title(display->xdg_toplevel, "Wayland Demo");

	wl_surface_commit(display->wl_surface);
	wl_display_roundtrip(display->wl_display); // Ensure all requests are processed

	// 5. Create wl_egl_window and initialize EGL
	display->wl_egl_window = wl_egl_window_create(display->wl_surface, display->width, display->height);
	if (!display->wl_egl_window) {
		fprintf(stderr, "Failed to create wl_egl_window.\n");
		return -1;
	}

	PFNEGLGETPLATFORMDISPLAYEXTPROC eglGetPlatformDisplayEXT =
		(PFNEGLGETPLATFORMDISPLAYEXTPROC)eglGetProcAddress("eglGetPlatformDisplayEXT");
	if (!eglGetPlatformDisplayEXT) {
		fprintf(stderr, "eglGetPlatformDisplayEXT not available.\n");
		return -1;
	}

	display->egl_display = eglGetPlatformDisplayEXT(EGL_PLATFORM_WAYLAND_KHR, (void *)display->wl_display, NULL);
	if (display->egl_display == EGL_NO_DISPLAY) {
		fprintf(stderr, "Failed to get EGL display with eglGetPlatformDisplayEXT.\n");
		return -1;
	}

	if (!eglInitialize(display->egl_display, NULL, NULL)) {
		fprintf(stderr, "Failed to initialize EGL.\n");
		return -1;
	}

	// 6. Choose EGL configuration
	EGLint cfg_attr[] = {
		EGL_SURFACE_TYPE, EGL_WINDOW_BIT,
		EGL_RENDERABLE_TYPE, EGL_OPENGL_ES2_BIT,
		EGL_RED_SIZE, 8,
		EGL_GREEN_SIZE, 8,
		EGL_BLUE_SIZE, 8,
		EGL_ALPHA_SIZE, 8,
		EGL_NONE};
	EGLConfig config;
	EGLint num;
	if (!eglChooseConfig(display->egl_display, cfg_attr, &config, 1, &num) || (num != 1)) {
		fprintf(stderr, "Failed to choose EGL config.\n");
		return -1;
	}

	// 7. Create EGL surface
	display->egl_surface = eglCreateWindowSurface(display->egl_display, config,
									(EGLNativeWindowType)display->wl_egl_window, NULL);
	if (display->egl_surface == EGL_NO_SURFACE) {
		fprintf(stderr, "Failed to create EGL window surface.\n");
		return -1;
	}

	// 8. Create EGL context
	EGLint ctx_attr[] = {EGL_CONTEXT_CLIENT_VERSION, 2, EGL_NONE};
	display->egl_context = eglCreateContext(display->egl_display, config, EGL_NO_CONTEXT, ctx_attr);
	if (display->egl_context == EGL_NO_CONTEXT) {
		fprintf(stderr, "Failed to create EGL context.\n");
		return -1;
	}

	return 0;
}

void wayland_display_deinit(WaylandDisplay *display) {
	if (!display) return;

	if (display->egl_display != EGL_NO_DISPLAY) {
		eglMakeCurrent(display->egl_display, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
		if (display->egl_surface != EGL_NO_SURFACE) {
			eglDestroySurface(display->egl_display, display->egl_surface);
		}
		if (display->egl_context != EGL_NO_CONTEXT) {
			eglDestroyContext(display->egl_display, display->egl_context);
		}
		eglTerminate(display->egl_display);
		display->egl_display = EGL_NO_DISPLAY;
	}

	if (display->wl_egl_window) {
		wl_egl_window_destroy(display->wl_egl_window);
		display->wl_egl_window = NULL;
	}

	if (display->xdg_toplevel) {
		xdg_toplevel_destroy(display->xdg_toplevel);
		display->xdg_toplevel = NULL;
	}

	if (display->xdg_surface) {
		xdg_surface_destroy(display->xdg_surface);
		display->xdg_surface = NULL;
	}

	if (display->wl_surface) {
		wl_surface_destroy(display->wl_surface);
		display->wl_surface = NULL;
	}

	if (display->wl_registry) {
		wl_registry_destroy(display->wl_registry);
		display->wl_registry = NULL;
	}

	if (display->wl_display) {
		wl_display_disconnect(display->wl_display);
		display->wl_display = NULL;
	}
}

void wayland_display_destroy(WaylandDisplay *display) {
	if (display) {
		wayland_display_deinit(display);
		free(display);
	}
}

void wayland_display_dispatch_events(WaylandDisplay *display) {
	while (wl_display_dispatch_pending(display->wl_display) != -1) {
		// Handle all pending events
	}
}

bool wayland_display_is_running(WaylandDisplay *display) {
	return display->running;
}

void wayland_display_make_current(WaylandDisplay *display) {
	if (!eglMakeCurrent(display->egl_display, display->egl_surface,
					   display->egl_surface, display->egl_context)) {
		fprintf(stderr, "Failed to make EGL context current.\n");
	}
}

void wayland_display_swap_buffers(WaylandDisplay *display) {
	eglSwapBuffers(display->egl_display, display->egl_surface);
	wl_display_flush(display->wl_display);
}
