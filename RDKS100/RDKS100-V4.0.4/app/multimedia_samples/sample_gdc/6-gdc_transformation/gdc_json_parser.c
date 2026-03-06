//----------------------------------------------------------------------------
//   The confidential and proprietary information contained in this file may
//   only be used by a person authorised under and to the extent permitted
//   by a subsisting licensing agreement from ARM Limited or its affiliates.
//
//          (C) COPYRIGHT [2017] ARM Limited or its affiliates.
//              ALL RIGHTS RESERVED
//
//   This entire notice must be reproduced on all copies of this file
//   and copies of this file may only be made by a person if such person is
//   permitted to do so under the terms of a subsisting license agreement
//   from ARM Limited or its affiliates.
//----------------------------------------------------------------------------
#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include "gdc_json_parser.h"
//---------------------------------------------------------
#ifdef _MSC_VER
#pragma warning(disable : 4706) //  assignment within conditional expression
#endif
//---------------------------------------------------------
static int names_match(const char *name1, const char *name2)
{
	return !strncmp(name1, name2, strlen(name2));
}
//---------------------------------------------------------
static int get_json_item(const char **buf, const char **name, const char **value)
{
	const char *ptr = *buf;
	int nbracket_cnt = 0;
	int cbracket_cnt = 0;
	int in_quotes = 0;
	// search name
	while (*ptr != '\"') {
		if (!isspace(*ptr)) {
			*buf = ptr;
			return 0;
		}
		ptr++;
	}
	ptr++;
	*name = ptr;
	// search end of name
	while (!(ptr[-1] == '\"' && *ptr == ':')) {
		if (!*ptr)
			return -3;
		ptr++;
	}
	ptr++;
	while (*ptr == ' ')
		ptr++;
	*value = ptr;
	while (*ptr && !(!nbracket_cnt && !cbracket_cnt && !in_quotes &&
			 !(isdigit(*ptr) || *ptr == '.' || *ptr == '{' || *ptr == '[' || *ptr == '\"'))) {
		if (*ptr == '\"')
			in_quotes = !in_quotes;
		if (!in_quotes) {
			if (*ptr == '[')
				nbracket_cnt++;
			if (*ptr == ']') {
				if (nbracket_cnt)
					nbracket_cnt--;
				else
					return -4;
			}
			if (*ptr == '{')
				cbracket_cnt++;
			if (*ptr == '}') {
				if (cbracket_cnt)
					cbracket_cnt--;
				else
					return -5;
			}
		}
		ptr++;
	}
	if (*ptr == ',')
		ptr++;
	*buf = ptr;
	return 1;
}
//---------------------------------------------------------
static const char *get_digit(const char *buf, const char **value)
{
	while (!(isdigit(*buf) || *buf == '-' || *buf == '.')) { // search start of digit
		if (*buf == '[' || *buf == ']' || *buf == '{' || *buf == '}')
			return NULL;
		buf++;
	}
	*value = buf;
	while (isdigit(*buf) || *buf == '.' || *buf == '-')
		buf++; // search end of digit
	return buf;
}
//---------------------------------------------------------
static void get_string(const char *buf, char *value)
{
	if (*buf != '\"') {
		return;
	}
	buf++;
	while (*buf != '\"') { // search start of digit
		*value++ = *buf++;
	}
	*value = '\0';
}
//---------------------------------------------------------
static void get_res(const char *buf, resolution_t *res)
{
	const char *width;
	const char *height;
	if (*buf++ != '[' || !(buf = get_digit(buf, &width)) || !(buf = get_digit(buf, &height))) {
		printf("wrong resolution format\n");
		return;
	}
	res->w = strtoul(width, NULL, 0);
	res->h = strtoul(height, NULL, 0);
}
//---------------------------------------------------------
static void get_param(const char *buf, param_t *param)
{
	const char *name;
	const char *value;
	if (*buf++ != '{') {
		printf("wrong param format\n");
		return;
	}
	while (get_json_item(&buf, &name, &value) > 0) {
		if (names_match(name, "fov")) {
			param->fov = atof(value);
		} else if (names_match(name, "offsetX")) {
			param->x_offset = strtoul(value, NULL, 0);
		} else if (names_match(name, "offsetY")) {
			param->y_offset = strtoul(value, NULL, 0);
		} else if (names_match(name, "diameter")) {
			param->diameter = strtoul(value, NULL, 0);
		}
	}
}
//---------------------------------------------------------
static void get_position(const char *buf, rect_t *pos)
{
	const char *x;
	const char *y;
	const char *w;
	const char *h;
	if (*buf++ != '[' || !(buf = get_digit(buf, &x)) || !(buf = get_digit(buf, &y)) ||
		!(buf = get_digit(buf, &w)) || !(buf = get_digit(buf, &h))) {
		printf("wrong position format\n");
		return;
	}
	pos->x = strtoul(x, NULL, 0) & ~1; // must be even
	pos->y = strtoul(y, NULL, 0) & ~1; // must be even
	pos->w = strtoul(w, NULL, 0) & ~1; // must be even
	pos->h = strtoul(h, NULL, 0) & ~1; // must be even
}
//---------------------------------------------------------
static void get_transformation(const char *value, transformation_t *transform)
{
	if (*value++ != '\"') {
		printf("wrong transformation format\n");
		return;
	}
	if (names_match(value, "Stereographic")) {
		*transform = STEREOGRAPHIC;
	} else if (names_match(value, "Cylindrical")) {
		*transform = CYLINDRICAL;
	} else if (names_match(value, "Panoramic")) {
		*transform = PANORAMIC;
	} else if (names_match(value, "Universal")) {
		*transform = UNIVERSAL;
	} else if (names_match(value, "Custom")) {
		*transform = CUSTOM;
	} else if (names_match(value, "Affine")) {
		*transform = AFFINE;
	} else if (names_match(value, "Dewarp_keystone")) {
		*transform = DEWARP_KEYSTONE;
	} else {
		printf("tranformation is not supported\n");
	}
}
//---------------------------------------------------------
static void get_ptz(const char *buf, window_t *wnd)
{
	const char *p;
	const char *t;
	const char *z;
	// default parameters
	wnd->pan = 0;
	wnd->tilt = 0;
	wnd->zoom = 1.0;
	if (*buf++ != '[' || !(buf = get_digit(buf, &p)) || !(buf = get_digit(buf, &t)) ||
		!(buf = get_digit(buf, &z))) {
		printf("wrong PTZ format\n");
		return;
	}
	wnd->pan = strtol(p, NULL, 0);
	wnd->tilt = strtol(t, NULL, 0);
	wnd->zoom = atof(z);
}
//---------------------------------------------------------
static void get_transformation_param(const char *buf, window_t *wnd)
{
	const char *name;
	const char *value;
	// all default parameters
	wnd->strength = 1.0;
	wnd->strengthY = 1.0;
	wnd->angle = 0;
	wnd->elevation = 0;
	wnd->azimuth = 0;
	wnd->keep_ratio = 1;
	wnd->FOV_h = 90;
	wnd->FOV_w = 90;
	wnd->cylindricity_y = 0;
	wnd->cylindricity_x = 0;
	wnd->trapezoid_left_angle = 90;
	wnd->trapezoid_right_angle = 90;
	if (*buf++ != '{')
		return;
	while (get_json_item(&buf, &name, &value) > 0) {
		if (names_match(name, "strengthY")) {
			wnd->strengthY = atof(value);
		} else if (names_match(name, "strength")) {
			wnd->strength = atof(value);
		} else if (names_match(name, "angle") || names_match(name, "rotation")) {
			wnd->angle = atof(value);
		} else if (names_match(name, "elevation")) {
			wnd->elevation = atof(value);
		} else if (names_match(name, "azimuth")) {
			wnd->azimuth = atof(value);
		} else if (names_match(name, "keepRatio")) {
			wnd->keep_ratio = atoi(value);
		} else if (names_match(name, "fovH")) {
			wnd->FOV_h = atof(value);
		} else if (names_match(name, "fovW")) {
			wnd->FOV_w = atof(value);
		} else if (names_match(name, "cylindricityY")) {
			wnd->cylindricity_y = atof(value);
		} else if (names_match(name, "cylindricityX")) {
			wnd->cylindricity_x = atof(value);
		} else if (names_match(name, "customTransformation")) {
			get_string(value, wnd->custom_file);
		} else if (names_match(name, "left_base_angle")) {
			wnd->trapezoid_left_angle = atof(value);
		} else if (names_match(name, "right_base_angle")) {
			wnd->trapezoid_right_angle = atof(value);
		}
	}
}
//---------------------------------------------------------
static void get_roi(const char *buf, window_t *wnd)
{
	const char *name;
	const char *value;
	// all default parameters
	wnd->input_roi_r.x = 0;
	wnd->input_roi_r.y = 0;
	wnd->input_roi_r.w = 0;
	wnd->input_roi_r.h = 0;
	if (*buf++ != '{')
		return;
	while (get_json_item(&buf, &name, &value) > 0) {
		if (names_match(name, "x")) {
			wnd->input_roi_r.x = atoi(value);
		} else if (names_match(name, "y")) {
			wnd->input_roi_r.y = atoi(value);
		} else if (names_match(name, "w")) {
			wnd->input_roi_r.w = atoi(value);
		} else if (names_match(name, "h")) {
			wnd->input_roi_r.h = atoi(value);
		}
	}
}
//---------------------------------------------------------
static int get_window(const char **buf, window_t *wnd)
{
	const char *ptr = *buf;
	const char *name;
	const char *value;
	while (*ptr != '{') {
		if (!isspace(*ptr)) {
			printf("wrong window format\n");
			return 0;
		}
		ptr++;
	}
	ptr++;
	while (get_json_item(&ptr, &name, &value) > 0) {
		if (names_match(name, "position")) {
			get_position(value, &wnd->out_r);
		} else if (names_match(name, "transformation")) {
			get_transformation(value, &wnd->transform);
		} else if (names_match(name, "param")) {
			get_transformation_param(value, wnd);
		} else if (names_match(name, "ptz")) {
			get_ptz(value, wnd);
		} else if (names_match(name, "roi")) {
			get_roi(value, wnd);
		}
	}
	*buf = ptr + 1;
	return 1;
}
//---------------------------------------------------------
static void get_windows(const char *buf, window_t **wnds, uint32_t *wnd_num)
{
	int ret;
	int wnd_cnt = 0;
	if (*buf++ != '[') {
		printf("wrong transformations format\n");
		return;
	}
	do {
		*wnds = (window_t *)realloc(*wnds, (wnd_cnt + 1) * sizeof(window_t));
		if ((ret = get_window(&buf, *wnds + wnd_cnt)) > 0) {
			wnd_cnt++;
			if (*buf == ',')
				buf++;
			else
				break;
		}
	} while (ret > 0);
	if (ret < 0) {
		wnd_cnt = 0; // error occures
		printf("Error during transformations parsing\n");
	}
	*wnd_num = wnd_cnt;
}
//---------------------------------------------------------
static int get_custom_points(FILE *cf, window_t *window)
{
	int x, y, i, retval = 1;
	int ch;
	char point[16] = { 0 };

	for (y = 0; y < (window->custom.h + 1); y++) {
		for (x = 0; x < (window->custom.w + 1); x++) {
			i = 0;
			//printf("==== x %d custom.w x h %dx%d ===\n", x, window->custom.w, window->custom.h);
			while ((ch = fgetc(cf)) != (int)':' && (ch != EOF)) {
				point[i++] = (char)ch;
			}
			if (ch == EOF) {
				printf("out1 check end x %d y %d  i %d.\n", x, y, i);
				goto out;
			}
			point[i] = '\0';
			//printf("x %d y %d i %d y pos %d ->point %lf\n", x, y, i, (y*(window->custom.w + 1) + x),  atof(point));
			window->custom.points[y * (window->custom.w + 1) + x].y = atof(point);
			i = 0;
			point[0] = '\0';
			while (((ch = fgetc(cf)) != (int)' ') && (ch != (int)'\n') && (ch != EOF)) {
				point[i++] = (char)ch;
			}
			if (ch == EOF) {
				printf("out2 check end x %d y %d  i %d.\n", x, y, i);
				goto out;
			}
			point[i] = '\0';
			//printf("x %d y%d i %d x pos %d ->point %lf\n", x, y, i, (y*(window->custom.w + 1) + x), atof(point));
			window->custom.points[y * (window->custom.w + 1) + x].x = atof(point);
		}
	}
out:
	if (x * y != (window->custom.h + 1) * (window->custom.w + 1)) {
		fprintf(stderr, "ERROR:No match data with size set in the custom file\n");
		retval = -1;
	} else {
		retval = 0;
	}

	return retval;
}
//---------------------------------------------------------
static int get_custom_size(FILE *cf, window_t *window)
{
	char ch;
	char size[16] = { 0 };
	int i = 0;

	// =================================
	// Read enable disable Full tile calculation
	while ((ch = (char)fgetc(cf)) != '\n') {
		size[i++] = ch;
	}
	size[i] = '\0';
	window->custom.full_tile_calc = atoi(size);
	// =================================

	// =================================
	// Read tile calculation pixel increment
	i = 0;
	size[0] = '\0';
	while ((ch = (char)fgetc(cf)) != ' ') {
		size[i++] = ch;
	}
	size[i] = '\0';
	window->custom.tile_incr_y = atoi(size);

	i = 0;
	size[0] = '\0';
	while ((ch = (char)fgetc(cf)) != '\n') {
		size[i++] = ch;
	}
	size[i] = '\0';
	window->custom.tile_incr_x = atoi(size);

	if (window->custom.full_tile_calc && (!window->custom.tile_incr_x || !window->custom.tile_incr_y)) {
		fprintf(stderr, "Tile increment should be bigger that 0 pixels!!!\n");
		abort();
	}
	// =================================

	// =================================
	i = 0;
	size[0] = '\0';
	// Read Custom grid size
	while ((ch = (char)fgetc(cf)) != ' ') {
		size[i++] = ch;
	}
	size[i] = '\0';
	window->custom.h = atoi(size) - 1;

	i = 0;
	size[0] = '\0';
	while ((ch = (char)fgetc(cf)) != '\n') {
		size[i++] = ch;
	}
	size[i] = '\0';
	window->custom.w = atoi(size) - 1;

	// =================================

	// =================================
	// Read Custom grid centre point
	i = 0;
	size[0] = '\0';
	while ((ch = (char)fgetc(cf)) != ' ') {
		size[i++] = ch;
	}
	size[i] = '\0';

	window->custom.centery = atof(size);
	i = 0;
	size[0] = '\0';
	while ((ch = (char)fgetc(cf)) != '\n') {
		size[i++] = ch;
	}
	size[i] = '\0';
	window->custom.centerx = atof(size);
	// =================================

	return 0;
}
//---------------------------------------------------------
static int parse_custom_transformation(FILE *cf, window_t *window)
{
	int retval = 0;
	retval |= get_custom_size(cf, window);
	window->custom.points = (point_t *)malloc((window->custom.h + 1) * (window->custom.w + 1) * sizeof(point_t));
	retval |= get_custom_points(cf, window);

	return retval;
}
//---------------------------------------------------------
static int get_frame_format(const char *value, frame_format_t *format)
{
	if ((value == NULL) || (format == NULL)) {
		return -1;
	}
	int res = 0;
	if (names_match(value, "\"luminance\"")) {
		*format = FMT_LUMINANCE;
	} else if (names_match(value, "\"planar444\"")) {
		*format = FMT_PLANAR_444;
	} else if (names_match(value, "\"planar420\"")) {
		*format = FMT_PLANAR_420;
	} else if (names_match(value, "\"semiplanar420\"")) {
		*format = FMT_SEMIPLANAR_420;
	} else {
		*format = FMT_UNKNOWN;
		res = -1;
	}
	return res;
}
//---------------------------------------------------------
ARM_GDC_API int gdc_parse_json(const char *buf, param_t *param, window_t **wnds, uint32_t *wnd_cnt)
{
	int ret = 0;
	const char *name;
	const char *value;
	size_t len = strlen(buf);
	if (buf[0] != '{' || buf[len - 1] != '}') {
		ret = -1;
	}
	buf++;
	while ((ret = get_json_item(&buf, &name, &value)) > 0) {
		if (names_match(name, "inputRes")) {
			get_res(value, &param->in);
		} else if (names_match(name, "outputRes")) {
			get_res(value, &param->out);
		} else if (names_match(name, "param")) {
			get_param(value, param);
		} else if (names_match(name, "transformations")) {
			get_windows(value, wnds, wnd_cnt);
		} else if (names_match(name, "mode")) {
			ret = get_frame_format(value, &param->format);
		}
	}

	for (uint32_t i = 0; i < *wnd_cnt; i++) {
		if ((*wnds)[i].transform == CUSTOM) {
			FILE *cf = fopen((*wnds)[i].custom_file, "r");
			if (!cf) {
				fprintf(stderr, "ERROR: Can't open custom transformation parameters file %s\n",
					(*wnds)[i].custom_file);
				return -2;
			}
			ret = parse_custom_transformation(cf, &(*wnds)[i]);
			fclose(cf);
		} else {
			(*wnds)[i].custom.full_tile_calc = 0;
		}
	}

	return ret;
}
//---------------------------------------------------------
ARM_GDC_API void gdc_parse_json_clean(window_t **wnds, uint32_t wnd_num)
{
	if (wnds != NULL) {
		window_t *wnd = *wnds;
		for (uint32_t i = 0; i < wnd_num; i++) {
			if (wnd[i].transform == CUSTOM && wnd[i].custom.points) {
				free(wnd[i].custom.points);
			}
		}
		free(*wnds);
		*wnds = NULL;
	}
}
//---------------------------------------------------------
/**
 * @NO{S09E03C02}
 * @ASIL{B}
 * @brief generate gdc config
 *
 * @param[in] const param_t *gdc_param: gdc parameter
 * @param[in] const window_t *windows: gdc processing window
 * @param[in] uint32_t wnd_num: gdc window number
 * @param[out] void **cfg_buf: gdb config binary buffer
 * @param[out] uint64_t *cfg_size: gdb config binary buffer size
 *
 * @retval zero: Success
 * @retval Less than zero: Fail, return error code
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
//coverity[misra_c_2012_rule_8_7_violation:SUPPRESS], ## violation reason SYSSW_V_8.7_02

int gdc_cfg_bin_gen(const char *layout_file, char *config_file, void **cfg_buf, uint64_t *config_size)
{
	int ret = 0;
	window_t *windows = NULL;
	uint32_t wnd_num = 0;
	param_t gdc_param;
	size_t json_sz;
	FILE *f = fopen(layout_file, "r");
	if (!f) {
		printf("Can't open transfo para file %s\n", layout_file);
		return -1;
	}

	fseek(f, 0, SEEK_END);
	long ftell_ret = ftell(f);

	if (ftell_ret == -1) {
		fclose(f);
		return -1;
	}
	else
		json_sz = ftell_ret;

	char *json = (char *)malloc(json_sz + 1);
	rewind(f);
	json[fread(json, sizeof(char), json_sz, f)] = 0;
	fclose(f);

	memset(&gdc_param, 0, sizeof(gdc_param));

	if (gdc_parse_json(json, &gdc_param, &windows, &wnd_num)) {
		printf("ERROR: Can't process json\n");
		gdc_parse_json_clean(&windows, wnd_num);
		return -1;
	}

	free(json);
	json = NULL;

	if (gdc_param.format == FMT_UNKNOWN) {
		printf("Can't process json: unknown frame format.\n");
		gdc_parse_json_clean(&windows, wnd_num);
		return -1;
	}

	if (wnd_num == 0) {
		printf("Warning: no windows are specified.\n");
		gdc_parse_json_clean(&windows, wnd_num);
		return -1;
	}

	ret = hbn_gen_gdc_cfg(&gdc_param, windows, wnd_num, cfg_buf, config_size);

	if (ret == 0 && config_file != NULL) {
		FILE *f = fopen(config_file, "wb");
		if (f != NULL) {
			fwrite(*cfg_buf, sizeof(char), *config_size, f);
			fclose(f);
		}
	}
	printf("gdc gen cfg_buf %p, size %lu \n", *cfg_buf, *config_size);
	gdc_parse_json_clean(&windows, wnd_num);
	return ret;
}
