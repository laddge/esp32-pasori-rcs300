#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "usb/usb_host.h"

#ifndef RCS300_H_
#define RCS300_H_

#define CMD_BUF_MAX 64
#define RESP_BUF_MAX 64

typedef struct {
  uint8_t cmd[CMD_BUF_MAX];
  size_t cmd_len;
  uint8_t resp[RESP_BUF_MAX];
  size_t resp_len;
  esp_err_t result;
} usb_cmd_request_t;

typedef struct {
  uint32_t actions;
  uint8_t dev_addr;
  usb_host_client_handle_t client_hdl;
  usb_device_handle_t dev_hdl;
} class_driver_ctrl_t;

typedef struct {
  uint8_t *resp;
  size_t resp_len;
  SemaphoreHandle_t sem;
  esp_err_t result;
} usb_xfer_context_t;

class RCS300 {
public:
  RCS300();

  uint8_t idm[8];
  uint8_t pmm[8];
  bool connected;
  class_driver_ctrl_t ctrl;

  void begin(void);
  esp_err_t usb_execute_command(const uint8_t *cmd, size_t cmd_len, uint8_t *resp, size_t *resp_len);
  esp_err_t polling(uint16_t systemCode = 0xffff);

private:
  usb_cmd_request_t req;
  usb_cmd_request_t *req_ptr;

  esp_err_t usb_transfer(usb_host_client_handle_t client_hdl, usb_device_handle_t dev_hdl);
};

#endif /* !RCS300_H_ */
