#include "Remote_OLED_display.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

/* ========================================== */
/* 内部静态变量 */
/* ========================================== */

static remote_oled_display_oled_data_t g_oled_data = {0};

/* ========================================== */
/* 基础功能实现 */
/* ========================================== */

/**
 * @brief 初始化
 */
int remote_oled_display_init(Remote_OLED_display_param_handle_t* handle){
    *handle = (Remote_OLED_display_param_handle_t)malloc(sizeof(remote_oled_display_param_handle));
    if(*handle == NULL) return -1;
    
    (*handle)->count = 0;
    (*handle)->auth_key[0] = 0;
    (*handle)->is_auth = 1;
    (*handle)->edit_cb = NULL;
    memset((*handle)->params, 0, REMOTE_OLED_DISPLAY_PARAMS_MAX * sizeof(remote_oled_display_param_t));
    
    return REMOTE_OLED_DISPLAY_ENUM_ERR_SUCCESS;
}

/**
 * @brief 设置认证密钥
 */
int remote_oled_display_set_key(Remote_OLED_display_param_handle_t handle, const char* key, size_t len){
    if(key == NULL){
        handle->is_auth = 1;
        handle->auth_key[0] = 0;
    }else{
        memset(handle->auth_key, 0, 16);
        memcpy(handle->auth_key, key, len > 15 ? 15 : len);
        handle->is_auth = 0;
    }
    return REMOTE_OLED_DISPLAY_ENUM_ERR_SUCCESS;
}

/**
 * @brief 注册参数
 */
int remote_oled_display_register(Remote_OLED_display_param_handle_t handle, remote_oled_display_param_t param){
    if(param->type >= REMOTE_OLED_DISPLAY_ENUM_TYPE_MAX) return -1;
    handle->params[handle->count] = param;
    handle->count++;
    return REMOTE_OLED_DISPLAY_ENUM_ERR_SUCCESS;
}

/**
 * @brief 处理协议数据
 */
int remote_oled_display_process(Remote_OLED_display_param_handle_t handle, uint8_t* in_buffer, size_t in_len, uint8_t* out_buffer, size_t* out_len) {
    int in_buffer_offset = 0;
    int out_buffer_offset = 0;
    
    if(in_len == 0) return REMOTE_OLED_DISPLAY_ENUM_ERR_DATA_TO_LITTLE;
    
    uint8_t cmd = in_buffer[in_buffer_offset++];
    if(cmd & REMOTE_OLED_DISPLAY_CMD_ACK_FLAG) return REMOTE_OLED_DISPLAY_ENUM_ERR_ILLEGAL_CMD;
    
    switch(cmd){
        case REMOTE_OLED_DISPLAY_ENUM_CMD_GET_COUNT:
        {
            if(!handle->is_auth){
                out_buffer[out_buffer_offset++] = REMOTE_OLED_DISPLAY_ENUM_CMD_NO_AUTH | REMOTE_OLED_DISPLAY_CMD_ACK;
                break;
            }
            out_buffer[out_buffer_offset++] = cmd | REMOTE_OLED_DISPLAY_CMD_ACK;
            out_buffer[out_buffer_offset++] = 0xFF & (handle->count >> 24);
            out_buffer[out_buffer_offset++] = 0xFF & (handle->count >> 16);
            out_buffer[out_buffer_offset++] = 0xFF & (handle->count >> 8);
            out_buffer[out_buffer_offset++] = 0xFF & (handle->count >> 0);
        }
        break;
        
        case REMOTE_OLED_DISPLAY_ENUM_CMD_GET_PARAM:
        {
            if(!handle->is_auth){
                out_buffer[out_buffer_offset++] = REMOTE_OLED_DISPLAY_ENUM_CMD_NO_AUTH | REMOTE_OLED_DISPLAY_CMD_ACK;
                break;
            }
            out_buffer[out_buffer_offset++] = cmd | REMOTE_OLED_DISPLAY_CMD_ACK;
            if(in_len < 2) return REMOTE_OLED_DISPLAY_ENUM_ERR_DATA_TO_LITTLE;
            
            uint8_t idx = in_buffer[in_buffer_offset++];
            if(idx >= handle->count) return REMOTE_OLED_DISPLAY_ENUM_ERR_OUT_OF_RANGE;
            
            remote_oled_display_param_t param = handle->params[idx];
            out_buffer[out_buffer_offset++] = idx;
            
            // name
            out_buffer[out_buffer_offset++] = 0xFF & strlen(param->name);
            memcpy(&out_buffer[out_buffer_offset], param->name, strlen(param->name));
            out_buffer_offset += strlen(param->name);
            
            // desc
            out_buffer[out_buffer_offset++] = 0xFF & strlen(param->desc);
            memcpy(&out_buffer[out_buffer_offset], param->desc, strlen(param->desc));
            out_buffer_offset += strlen(param->desc);
            
            // privilege
            out_buffer[out_buffer_offset++] = param->priv;
            
            // type
            out_buffer[out_buffer_offset++] = param->type;
            
            // data
            switch(param->type){
                case REMOTE_OLED_DISPLAY_ENUM_TYPE_UINT8:
                {
                    out_buffer[out_buffer_offset++] = *((uint8_t*)param->data);
                }
                break;
                case REMOTE_OLED_DISPLAY_ENUM_TYPE_INTEGER:
                {
                    out_buffer[out_buffer_offset+0] = *((int*)param->data) >> 24;
                    out_buffer[out_buffer_offset+1] = *((int*)param->data) >> 16;
                    out_buffer[out_buffer_offset+2] = *((int*)param->data) >> 8;
                    out_buffer[out_buffer_offset+3] = *((int*)param->data) >> 0;
                    out_buffer_offset += sizeof(int);
                }
                break;
                case REMOTE_OLED_DISPLAY_ENUM_TYPE_FLOAT:
                {
                    memcpy(&out_buffer[out_buffer_offset], param->data, sizeof(float));
                    out_buffer_offset += sizeof(float);
                }
                break;
                case REMOTE_OLED_DISPLAY_ENUM_TYPE_UINT8_ARRAY:
                case REMOTE_OLED_DISPLAY_ENUM_TYPE_STR:
                {
                    out_buffer[out_buffer_offset++] = param->size;
                    memset(&out_buffer[out_buffer_offset], 0, param->size);
                    memcpy(&out_buffer[out_buffer_offset], param->data, param->size);
                    out_buffer_offset += param->size;
                }
                break;
                default:
                break;
            }
        }
        break;
        
        case REMOTE_OLED_DISPLAY_ENUM_CMD_AUTH:
        {
            if(handle->auth_key[0] == 0){
                out_buffer[out_buffer_offset++] = REMOTE_OLED_DISPLAY_ENUM_CMD_AUTH_PASS | REMOTE_OLED_DISPLAY_CMD_ACK;
                handle->is_auth = 1;
                break;
            }
            if(in_len < 2){
                out_buffer[out_buffer_offset++] = REMOTE_OLED_DISPLAY_ENUM_CMD_AUTH_FAIL | REMOTE_OLED_DISPLAY_CMD_ACK;
                break;
            }
            
            uint8_t auth_key_len = in_buffer[in_buffer_offset++];
            
            if(auth_key_len != strlen(handle->auth_key)){
                out_buffer[out_buffer_offset++] = REMOTE_OLED_DISPLAY_ENUM_CMD_AUTH_FAIL | REMOTE_OLED_DISPLAY_CMD_ACK;
                break;
            }
            
            if(memcmp(&in_buffer[in_buffer_offset], handle->auth_key, auth_key_len)){
                out_buffer[out_buffer_offset++] = REMOTE_OLED_DISPLAY_ENUM_CMD_AUTH_FAIL | REMOTE_OLED_DISPLAY_CMD_ACK;
                break;
            }
            
            out_buffer[out_buffer_offset++] = REMOTE_OLED_DISPLAY_ENUM_CMD_AUTH_PASS | REMOTE_OLED_DISPLAY_CMD_ACK;
            handle->is_auth = 1;
        }
        break;
        
        default:
            out_buffer[out_buffer_offset++] = REMOTE_OLED_DISPLAY_ENUM_CMD_ERROR;
            *out_len = out_buffer_offset;
            return REMOTE_OLED_DISPLAY_ENUM_ERR_ILLEGAL_CMD;
    }
    
    *out_len = out_buffer_offset;
    return REMOTE_OLED_DISPLAY_ENUM_ERR_SUCCESS;
}

/**
 * @brief 十六进制调试输出
 */
void remote_oled_display_hexdump(const char* tag, const void* pdata, int len) {
#ifdef CONFIG_REMOTE_OLED_DISPLAY_HEXDUMP
    int i, j, k;
    const char* data = (const char*)pdata;
    char buf[256], str[64], t[] = "0123456789ABCDEF";
    
    for (i = j = k = 0; i < len; i++) {
        if (0 == i % 16) 
            j += sprintf(buf + j, "[ %s ] %04xh: ", tag, i); 
        buf[j++] = t[0x0f & (data[i] >> 4)];
        buf[j++] = t[0x0f & data[i]];
        buf[j++] = ' ';
        str[k++] = isprint(data[i]) ? data[i] : '.';
        if (0 == (i + 1) % 16) {
            str[k] = 0;
            j += sprintf(buf + j, "| %s\n", str);
            printf("%s", buf);
            j = k = buf[0] = str[0] = 0;
        }
    }
    str[k] = 0;
    if (k) {
        for (int l = 0; l < 3 * (16 - k); l++)
            buf[j++] = ' ';
        j += sprintf(buf + j, "| %s\n", str);
    }   
    if (buf[0]) printf("%s", buf);
#endif
}

/* ========================================== */
/* OLED显示功能实现 */
/* ========================================== */

/**
 * @brief 初始化OLED显示参数（注册12个只读参数）
 */
int remote_oled_display_oled_display_init(Remote_OLED_display_param_handle_t handle){
    if(handle == NULL) return -1;
    
    // 参数1: 转角
    static remote_oled_display_param param_angle = {
        "ang", "degree", REMOTE_OLED_DISPLAY_ENUM_PRIVILEGE_RO, 
        REMOTE_OLED_DISPLAY_ENUM_TYPE_FLOAT, 1, &g_oled_data.angle
    };
    remote_oled_display_register(handle, &param_angle);
    
    // 参数2: 中心偏移
    static remote_oled_display_param param_center = {
        "ctr", "percent", REMOTE_OLED_DISPLAY_ENUM_PRIVILEGE_RO, 
        REMOTE_OLED_DISPLAY_ENUM_TYPE_FLOAT, 1, &g_oled_data.center_percent
    };
    remote_oled_display_register(handle, &param_center);
    
    // 参数3: 左侧曲率
    static remote_oled_display_param param_l_curv = {
        "l_cv", "x100", REMOTE_OLED_DISPLAY_ENUM_PRIVILEGE_RO, 
        REMOTE_OLED_DISPLAY_ENUM_TYPE_FLOAT, 1, &g_oled_data.left_curvature
    };
    remote_oled_display_register(handle, &param_l_curv);
    
    // 参数4: 右侧曲率
    static remote_oled_display_param param_r_curv = {
        "r_cv", "x100", REMOTE_OLED_DISPLAY_ENUM_PRIVILEGE_RO, 
        REMOTE_OLED_DISPLAY_ENUM_TYPE_FLOAT, 1, &g_oled_data.right_curvature
    };
    remote_oled_display_register(handle, &param_r_curv);
    
    // 参数5: 拟合类型
    static remote_oled_display_param param_fit_type = {
        "f_tp", "type", REMOTE_OLED_DISPLAY_ENUM_PRIVILEGE_RO, 
        REMOTE_OLED_DISPLAY_ENUM_TYPE_FLOAT, 1, &g_oled_data.left_fit_type
    };
    remote_oled_display_register(handle, &param_fit_type);
    
    // 参数6: R²值
    static remote_oled_display_param param_r2 = {
        "r_sq", "percent", REMOTE_OLED_DISPLAY_ENUM_PRIVILEGE_RO, 
        REMOTE_OLED_DISPLAY_ENUM_TYPE_FLOAT, 1, &g_oled_data.left_r_squared
    };
    remote_oled_display_register(handle, &param_r2);
    
    // 参数7: 30度距离
    static remote_oled_display_param param_d30 = {
        "d30", "cm", REMOTE_OLED_DISPLAY_ENUM_PRIVILEGE_RO, 
        REMOTE_OLED_DISPLAY_ENUM_TYPE_FLOAT, 1, &g_oled_data.distance_30
    };
    remote_oled_display_register(handle, &param_d30);
    
    // 参数8: 330度距离
    static remote_oled_display_param param_d330 = {
        "d330", "cm", REMOTE_OLED_DISPLAY_ENUM_PRIVILEGE_RO, 
        REMOTE_OLED_DISPLAY_ENUM_TYPE_FLOAT, 1, &g_oled_data.distance_330
    };
    remote_oled_display_register(handle, &param_d330);
    
    // 参数9: 速度
    static remote_oled_display_param param_spd = {
        "spd", "speed", REMOTE_OLED_DISPLAY_ENUM_PRIVILEGE_RO, 
        REMOTE_OLED_DISPLAY_ENUM_TYPE_INTEGER, 1, &g_oled_data.speed
    };
    remote_oled_display_register(handle, &param_spd);
    
    // 参数10: U弯状态
    static remote_oled_display_param param_ustate = {
        "u_st", "state", REMOTE_OLED_DISPLAY_ENUM_PRIVILEGE_RO, 
        REMOTE_OLED_DISPLAY_ENUM_TYPE_INTEGER, 1, &g_oled_data.u_turn_state
    };
    remote_oled_display_register(handle, &param_ustate);
    
    // 参数11: 曲率弯道状态
    static remote_oled_display_param param_cstate = {
        "c_st", "state", REMOTE_OLED_DISPLAY_ENUM_PRIVILEGE_RO, 
        REMOTE_OLED_DISPLAY_ENUM_TYPE_INTEGER, 1, &g_oled_data.curvature_turn_state
    };
    remote_oled_display_register(handle, &param_cstate);
    
    // 参数12: 减速状态
    static remote_oled_display_param param_dstate = {
        "d_st", "state", REMOTE_OLED_DISPLAY_ENUM_PRIVILEGE_RO, 
        REMOTE_OLED_DISPLAY_ENUM_TYPE_INTEGER, 1, &g_oled_data.deceleration_state
    };
    remote_oled_display_register(handle, &param_dstate);
    
    return REMOTE_OLED_DISPLAY_ENUM_ERR_SUCCESS;
}

/**
 * @brief 更新OLED显示数据
 */
int remote_oled_display_oled_display_update(Remote_OLED_display_param_handle_t handle, remote_oled_display_oled_data_t* data){
    if(handle == NULL || data == NULL) return -1;
    
    // 直接拷贝整个结构体
    memcpy(&g_oled_data, data, sizeof(remote_oled_display_oled_data_t));
    
    return REMOTE_OLED_DISPLAY_ENUM_ERR_SUCCESS;
}