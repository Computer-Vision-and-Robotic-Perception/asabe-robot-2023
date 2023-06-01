#include "arducam_mipicamera.h"
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#define LOG(fmt, args...) fprintf(stderr, fmt "\n", ##args)
#define SOFTWARE_AE_AWB
#define ENABLE_PREVIEW

FILE *fd, *fd2;
int frame_count = 0;
int frame_count2 = 0;
int video_callback(BUFFER *buffer) {
    if (TIME_UNKNOWN == buffer->pts) {
        // Frame data in the second half
    }
    // LOG("buffer length = %d, pts = %llu, flags = 0x%X", buffer->length, buffer->pts, buffer->flags);

    if (buffer->length) {
        if (buffer->flags & MMAL_BUFFER_HEADER_FLAG_CONFIG) {
            // SPS PPS
            if (fd) {
                fwrite(buffer->data, 1, buffer->length, fd);
                fflush(fd);
            }
        }
        if (buffer->flags & MMAL_BUFFER_HEADER_FLAG_CODECSIDEINFO) {
            /// Encoder outputs inline Motion Vectors
        } else {
            // MMAL_BUFFER_HEADER_FLAG_KEYFRAME
            // MMAL_BUFFER_HEADER_FLAG_FRAME_END
            if (fd) {
                int bytes_written = fwrite(buffer->data, 1, buffer->length, fd);
                fflush(fd);
            }
            // Here may be just a part of the data, we need to check it.
            if (buffer->flags & MMAL_BUFFER_HEADER_FLAG_FRAME_END)
                frame_count++;
        }
    }
    return 0;
}

int video_callback2(BUFFER *buffer) {
    if (TIME_UNKNOWN == buffer->pts) {
        // Frame data in the second half
    }
    // LOG("buffer length = %d, pts = %llu, flags = 0x%X", buffer->length, buffer->pts, buffer->flags);

    if (buffer->length) {
        if (buffer->flags & MMAL_BUFFER_HEADER_FLAG_CONFIG) {
            // SPS PPS
            if (fd2) {
                fwrite(buffer->data, 1, buffer->length, fd2);
                fflush(fd2);
            }
        }
        if (buffer->flags & MMAL_BUFFER_HEADER_FLAG_CODECSIDEINFO) {
            /// Encoder outputs inline Motion Vectors
        } else {
            // MMAL_BUFFER_HEADER_FLAG_KEYFRAME
            // MMAL_BUFFER_HEADER_FLAG_FRAME_END
            if (fd2) {
                int bytes_written = fwrite(buffer->data, 1, buffer->length, fd2);
                fflush(fd2);
            }
            // Here may be just a part of the data, we need to check it.
            if (buffer->flags & MMAL_BUFFER_HEADER_FLAG_FRAME_END)
                frame_count2++;
        }
    }
    return 0;
}

static void default_status(VIDEO_ENCODER_STATE *state) {
    // Default everything to zero
    memset(state, 0, sizeof(VIDEO_ENCODER_STATE));
    state->encoding = VIDEO_ENCODING_H264;
    state->bitrate = 17000000;
    state->immutableInput = 1; // Not working
    /**********************H264 only**************************************/
    state->intraperiod = -1;                  // Not set
                                              // Specify the intra refresh period (key frame rate/GoP size).
                                              // Zero to produce an initial I-frame and then just P-frames.
    state->quantisationParameter = 0;         // Quantisation parameter. Use approximately 10-40. Default 0 (off)
    state->profile = VIDEO_PROFILE_H264_HIGH; // Specify H264 profile to use for encoding
    state->level = VIDEO_LEVEL_H264_4;        // Specify H264 level to use for encoding
    state->bInlineHeaders = 0;                // Insert inline headers (SPS, PPS) to stream
    state->inlineMotionVectors = 0;           // output motion vector estimates
    state->intra_refresh_type = -1;           // Set intra refresh type
    state->addSPSTiming = 0;                  // zero or one
    state->slices = 1;
    /**********************H264 only**************************************/
}

int main(int argc, char **argv) {
    CAMERA_INSTANCE camera_instance, camera_instance2;
    int count = 0;
    int width = 0, height = 0;

    LOG("Open camera...");
    struct camera_interface cam_interface = {
        .i2c_bus = 0,           // /dev/i2c-0  or /dev/i2c-1   
        .camera_num = 0,        // mipi interface num
        .sda_pins = {28, 0},    // enable sda_pins[camera_num], disable sda_pins[camera_num ? 0 : 1]
        .scl_pins = {29, 1},    // enable scl_pins[camera_num], disable scl_pins[camera_num ? 0 : 1]
        .led_pins = {30, 2},
        .shutdown_pins ={31, 3},
    };
    int res = arducam_init_camera2(&camera_instance, cam_interface);
    if (res) {
        LOG("init camera status = %d", res);
        return -1;
    }

    width = 640*2;
    height = 480;
    LOG("Setting the resolution...");
    res = arducam_set_resolution(camera_instance, &width, &height);
    if (res) {
        LOG("set resolution status = %d", res);
        return -1;
    } else {
        LOG("Current resolution is %dx%d", width, height);
        LOG("Notice:You can use the list_format sample program to see the resolution and control supported by the camera.");
    }
    
#if defined(ENABLE_PREVIEW)
    LOG("Start preview...");
    PREVIEW_PARAMS preview_params = {
        .fullscreen = 0,             // 0 is use previewRect, non-zero to use full screen
        .opacity = 255,              // Opacity of window - 0 = transparent, 255 = opaque
        .window = {0, 0, 1280, 720}, // Destination rectangle for the preview window.
    };
    res = arducam_start_preview(camera_instance, &preview_params);
    if (res) {
        LOG("start preview status = %d", res);
        return -1;
    }
#endif

#if defined(SOFTWARE_AE_AWB)
    LOG("Enable Software Auto Exposure...");
    arducam_software_auto_exposure(camera_instance, 1);
    LOG("Enable Software Auto White Balance...");
    arducam_software_auto_white_balance(camera_instance, 1);
#endif

    fd = fopen("test.h264", "wb");
    VIDEO_ENCODER_STATE video_state;
    default_status(&video_state);
    // start video callback
    // Set video_state to NULL, using default parameters
    LOG("Start video encoding...");
    res = arducam_set_video_callback(camera_instance, &video_state, video_callback, NULL);
    if (res) {
        LOG("Failed to start video encoding, probably due to resolution greater than 1920x1080 or video_state setting error.");
        return -1;
    }

    cam_interface.camera_num = 1;
    res = arducam_init_camera2(&camera_instance2, cam_interface);
    if (res) {
        LOG("init camera status = %d", res);
        return -1;
    }

    width = 640*2;
    height = 480;
    LOG("Setting the resolution...");
    res = arducam_set_resolution(camera_instance2, &width, &height);
    if (res) {
        LOG("set resolution status = %d", res);
        return -1;
    } else {
        LOG("Current resolution is %dx%d", width, height);
        LOG("Notice:You can use the list_format sample program to see the resolution and control supported by the camera.");
    }
#if defined(ENABLE_PREVIEW)
    LOG("Start preview...");
    PREVIEW_PARAMS preview_params2 = {
        .fullscreen = 0,             // 0 is use previewRect, non-zero to use full screen
        .opacity = 255,              // Opacity of window - 0 = transparent, 255 = opaque
        .window = {0, 480, 640, 480}, // Destination rectangle for the preview window.
    };
res = arducam_start_preview(camera_instance2, &preview_params2);
    if (res) {
        LOG("start preview status = %d", res);
        return -1;
    }
#endif
#if defined(SOFTWARE_AE_AWB)
    LOG("Enable Software Auto Exposure...");
    arducam_software_auto_exposure(camera_instance2, 1);
    LOG("Enable Software Auto White Balance...");
    arducam_software_auto_white_balance(camera_instance2, 1);
#endif
fd2 = fopen("test2.h264", "wb");
 //   VIDEO_ENCODER_STATE video_state;
    default_status(&video_state);
    // start video callback
    // Set video_state to NULL, using default parameters
    LOG("Start video encoding...");
    res = arducam_set_video_callback(camera_instance2, &video_state, video_callback2, NULL);
    if (res) {
        LOG("Failed to start video encoding, probably due to resolution greater than 1920x1080 or video_state setting error.");
        return -1;
    }


    struct timespec start, end;
    clock_gettime(CLOCK_REALTIME, &start);
    usleep(1000 * 1000 * 10);
    clock_gettime(CLOCK_REALTIME, &end);

    double timeElapsed = (end.tv_sec - start.tv_sec) + (end.tv_nsec - start.tv_nsec) / 1000000000.0;
    LOG("Total frame count = %d", frame_count);
    LOG("TimeElapsed = %f", timeElapsed);

    // stop video callback
    LOG("Stop video encoding...");
    arducam_set_video_callback(camera_instance, NULL, NULL, NULL);
    fclose(fd);
    arducam_set_video_callback(camera_instance2, NULL, NULL, NULL);
    fclose(fd2);

    LOG("Close camera...");
    res = arducam_close_camera(camera_instance);
    if (res) {
        LOG("close camera status = %d", res);
    }
    res = arducam_close_camera(camera_instance2);
    if (res) {
        LOG("close camera status = %d", res);
    }
    return 0;
}