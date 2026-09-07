#ifndef __REMOTE_OLED_DISPLAY_H__
#define __REMOTE_OLED_DISPLAY_H__

#include <stdint.h>
#include <stddef.h>

/* ========================================== */
/* 配置定义 */
/* ========================================== */

#define REMOTE_OLED_DISPLAY_BUFFER_SIZE 128
#define REMOTE_OLED_DISPLAY_PARAMS_MAX 32
#define REMOTE_OLED_DISPLAY_PARAMS_NAME_MAX 16
#define REMOTE_OLED_DISPLAY_PARAMS_DESC_MAX 32

/* ========================================== */
/* 枚举定义 */
/* ========================================== */

// 错误码
typedef enum {
    REMOTE_OLED_DISPLAY_ENUM_ERR_SUCCESS = 0,
    REMOTE_OLED_DISPLAY_ENUM_ERR_ILLEGAL_CMD = -1,
    REMOTE_OLED_DISPLAY_ENUM_ERR_DATA_TO_LITTLE = -2,
    REMOTE_OLED_DISPLAY_ENUM_ERR_OUT_OF_RANGE = -3,
    REMOTE_OLED_DISPLAY_ENUM_ERR_READ_ONLY = -4,
    REMOTE_OLED_DISPLAY_ENUM_ERR_ILLEGAL_TYPE = -5,
    REMOTE_OLED_DISPLAY_ENUM_ERR_NO_TRANSMIT_FUNC = -6,
    REMOTE_OLED_DISPLAY_ENUM_ERR_AUTH_FAILURE = -7,
    REMOTE_OLED_DISPLAY_ENUM_ERR_UNKNOW = -8
} remote_oled_display_enum_err;

// 命令类型
typedef enum {
    REMOTE_OLED_DISPLAY_ENUM_CMD_ERROR = 0x00,
    REMOTE_OLED_DISPLAY_ENUM_CMD_GET_COUNT = 0x10,
    REMOTE_OLED_DISPLAY_ENUM_CMD_GET_PARAM = 0x11,
    REMOTE_OLED_DISPLAY_ENUM_CMD_SET_PARAM = 0x12,
    REMOTE_OLED_DISPLAY_ENUM_CMD_AUTH = 0x20,
    REMOTE_OLED_DISPLAY_ENUM_CMD_AUTH_PASS = 0x21,
    REMOTE_OLED_DISPLAY_ENUM_CMD_AUTH_FAIL = 0x22,
    REMOTE_OLED_DISPLAY_ENUM_CMD_NO_AUTH = 0x23
} remote_oled_display_enum_cmd;

// 参数权限
typedef enum {
    REMOTE_OLED_DISPLAY_ENUM_PRIVILEGE_RO = 0x00,  // 只读
    REMOTE_OLED_DISPLAY_ENUM_PRIVILEGE_RW = 0x01   // 读写
} remote_oled_display_enum_privilege;

// 参数类型
typedef enum {
    REMOTE_OLED_DISPLAY_ENUM_TYPE_UINT8 = 0x00,
    REMOTE_OLED_DISPLAY_ENUM_TYPE_INTEGER = 0x01,
    REMOTE_OLED_DISPLAY_ENUM_TYPE_FLOAT = 0x02,
    REMOTE_OLED_DISPLAY_ENUM_TYPE_UINT8_ARRAY = 0x03,
    REMOTE_OLED_DISPLAY_ENUM_TYPE_STR = 0x04,
    REMOTE_OLED_DISPLAY_ENUM_TYPE_MAX
} remote_oled_display_enum_type;

#define REMOTE_OLED_DISPLAY_CMD_ACK_FLAG 0x80
#define REMOTE_OLED_DISPLAY_CMD_ACK REMOTE_OLED_DISPLAY_CMD_ACK_FLAG

/* ========================================== */
/* 结构体定义 */
/* ========================================== */

// 参数定义结构
typedef struct {
    char name[REMOTE_OLED_DISPLAY_PARAMS_NAME_MAX];
    char desc[REMOTE_OLED_DISPLAY_PARAMS_DESC_MAX];
    remote_oled_display_enum_privilege priv;
    remote_oled_display_enum_type type;
    size_t size;
    void* data;
} remote_oled_display_param;

typedef remote_oled_display_param* remote_oled_display_param_t;

// 参数句柄结构
typedef struct {
    remote_oled_display_param_t params[REMOTE_OLED_DISPLAY_PARAMS_MAX];
    int count;
    char auth_key[16];
    int is_auth;
    int (*edit_cb)(remote_oled_display_param_t param);
} remote_oled_display_param_handle;

typedef remote_oled_display_param_handle* Remote_OLED_display_param_handle_t;

/* ========================================== */
/* OLED显示数据结构 */
/* ========================================== */

typedef struct {
    float angle;              // 转角
    float center_percent;     // 中心偏移
    float left_curvature;     // 左侧曲率
    float right_curvature;    // 右侧曲率
    float left_fit_type;      // 左侧拟合类型
    float left_r_squared;     // 左侧R²值
    float distance_30;        // 30度距离
    float distance_330;       // 330度距离
    int speed;                // 当前速度
    int u_turn_state;         // U弯状态
    int curvature_turn_state; // 曲率弯道状态
    int deceleration_state;   // 减速状态
} remote_oled_display_oled_data_t;

/* ========================================== */
/* 函数声明 */
/* ========================================== */

/**
 * @brief 初始化远程OLED显示模块
 */
int remote_oled_display_init(Remote_OLED_display_param_handle_t* handle);

/**
 * @brief 设置认证密钥
 */
int remote_oled_display_set_key(Remote_OLED_display_param_handle_t handle, const char* key, size_t len);

/**
 * @brief 注册参数
 */
int remote_oled_display_register(Remote_OLED_display_param_handle_t handle, remote_oled_display_param_t param);

/**
 * @brief 初始化OLED显示参数
 */
int remote_oled_display_oled_display_init(Remote_OLED_display_param_handle_t handle);

/**
 * @brief 更新OLED显示数据
 */
int remote_oled_display_oled_display_update(Remote_OLED_display_param_handle_t handle, remote_oled_display_oled_data_t* data);

/**
 * @brief 处理接收到的数据
 */
int remote_oled_display_process(Remote_OLED_display_param_handle_t handle, uint8_t* in_buffer, size_t in_len, uint8_t* out_buffer, size_t* out_len);

/**
 * @brief 十六进制数据调试输出
 */
void remote_oled_display_hexdump(const char* tag, const void* pdata, int len);

#endif /* __REMOTE_OLED_DISPLAY_H__ */