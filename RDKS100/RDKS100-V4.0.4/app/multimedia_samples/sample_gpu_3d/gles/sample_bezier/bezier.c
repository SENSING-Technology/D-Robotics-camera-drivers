#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "bmp.h"
#include "wayland_display.h"
#include <GLES2/gl2.h>
#define WIDTH 800
#define HEIGHT 600
// Vertex Shader
const char* vertexShaderSource = R"(
	attribute vec2 a_Position;
	void main() {
		gl_Position = vec4(a_Position, 0.0, 1.0);
	}
)";

// Fragment Shader
const char* fragmentShaderSource = R"(
	precision mediump float;
	void main() {
		gl_FragColor = vec4(1.0, 0.0, 0.0, 1.0); // Red color
	}
)";

// Control points for Bézier curve
float controlPoints[] = { -0.8f, -0.8f, 0.0f, 0.8f, 0.8f, -0.8f };

void calculateBezierPoints(float* points, int segments, float* controlPoints) {
	float t, x, y;
	for (int i = 0; i <= segments; ++i) {
		t = (float)i / segments;
		float u = 1 - t;

		// Quadratic Bézier: B(t) = (1-t)^2 * P0 + 2(1-t)t * P1 + t^2 * P2
		x = u * u * controlPoints[0] +
			2 * u * t * controlPoints[2] +
			t * t * controlPoints[4];
		y = u * u * controlPoints[1] +
			2 * u * t * controlPoints[3] +
			t * t * controlPoints[5];

		points[i * 2] = x;
		points[i * 2 + 1] = y;
	}
}
// Shader helper
GLuint create_shader(const char *source, GLenum shader_type) {
	GLuint shader = glCreateShader(shader_type);
	glShaderSource(shader, 1, &source, NULL);
	glCompileShader(shader);
	GLint compiled;
	glGetShaderiv(shader, GL_COMPILE_STATUS, &compiled);
	if (!compiled) {
		char buf[512];
		glGetShaderInfoLog(shader, sizeof(buf), NULL, buf);
		fprintf(stderr, "Shader compile failed: %s\n", buf);
		exit(1);
	}
	return shader;
}

int main() {

	// Init wayland
	WaylandDisplay* way_disp = wayland_display_create(WIDTH, HEIGHT); //export WAYLAND_DISPLAY=/run/user/1000/wayland-0
	if(way_disp == 0){
		printf("wayland_display_init failed\n");
		return -1;
	}
	int ret = wayland_display_init(way_disp);
	if(ret != 0){
		printf("wayland_display_init failed\n");
		return -1;
	}
	wayland_display_make_current(way_disp);

	// Setup OpenGL
	glViewport(0, 0, WIDTH, HEIGHT);
	glClearColor(1.0, 1.0, 1.0, 1.0);
	glClear(GL_COLOR_BUFFER_BIT);

	// Create simple shader
	GLuint vshader = create_shader(vertexShaderSource, GL_VERTEX_SHADER);
	GLuint fshader = create_shader(fragmentShaderSource, GL_FRAGMENT_SHADER);

	GLuint program = glCreateProgram();
	glAttachShader(program, vshader);
	glAttachShader(program, fshader);
	glLinkProgram(program);
	glUseProgram(program);

	// Calculate Bézier curve points
	int segments = 100;
	float* points = (float*)malloc(segments * 2 * sizeof(float));
	calculateBezierPoints(points, segments, controlPoints);

	// Create Vertex Buffer Object(VBO)
	GLuint vbo;
	glGenBuffers(1, &vbo);
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glBufferData(GL_ARRAY_BUFFER, segments * 2 * sizeof(float), points, GL_STATIC_DRAW);

	// Set Vertex Attribute for VBO
	GLint positionLocation = glGetAttribLocation(program, "a_Position");
	glEnableVertexAttribArray(positionLocation);
	glVertexAttribPointer(positionLocation, 2, GL_FLOAT, GL_FALSE, 0, 0);

	glDrawArrays(GL_LINE_STRIP, 0, segments);

	glFinish(); // important!

	// Save to file
	unsigned char *pixels = malloc(WIDTH * HEIGHT * 4);
	glReadPixels(0, 0, WIDTH, HEIGHT, GL_RGBA, GL_UNSIGNED_BYTE, pixels);
	ret = SaveBmpImage("bezier.bmp", WIDTH, HEIGHT, pixels);
	if(ret != 0){
		printf("Saved bezier.bmp failed.\n");

	}else{
		printf("Saved bezier.bmp\n");
	}

	// Displayed on the monitor
	wayland_display_swap_buffers(way_disp);
	sleep(10);

	// Cleanup
	free(pixels);
	glDeleteProgram(program);
	glDeleteBuffers(1, &vbo);
	wayland_display_deinit(way_disp);
	wayland_display_destroy(way_disp);
	return 0;
}
