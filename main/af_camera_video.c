#include "af_camera_video.h"

esp_err_t init_camera(camera_config_t* cam_config)
{
    //initialize the camera
    esp_err_t err = esp_camera_init(cam_config);
    if (err != ESP_OK)
    {
        return err;
    }
    return ESP_OK;
}

void SplitIntoChunks(uint8_t* chunk_buffer ,uint8_t* framebuffer, int start_index, int end_index, uint16_t* frameNumber, uint8_t chunkNumber,uint16_t* chunk_count)
{
    //little endian
    chunk_buffer[0]=(uint8_t)(chunk_count[0] & 0xFF);
    chunk_buffer[1]=(uint8_t)((chunk_count[0]>>8) & 0xFF);
    chunk_buffer[2]=(uint8_t)(frameNumber[0] & 0xFF);
    chunk_buffer[3]=(uint8_t)((frameNumber[0]>>8) & 0xFF);
    chunk_buffer[4]=chunkNumber;


    for(int i=CHUNK_HEADER_SIZE,j=start_index;j<end_index;i++,j++){
        chunk_buffer[i] = framebuffer[j];
    }

}