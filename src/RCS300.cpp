#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_err.h"
#include "usb/usb_host.h"

#include "RCS300.h"

#define TAG "RCS300"
#define RCS300_VID 0x054c
#define RCS300_PID 0x0dc9
#define RCS300_EP 0x02

#define CLASS_DRIVER_ACTION_OPEN_DEV    0x01
#define CLASS_DRIVER_ACTION_CLOSE_DEV   0x02

SemaphoreHandle_t sem_host_ready;

void daemon_task(void* pvParameters) {
  RCS300* self = static_cast<RCS300*>(pvParameters);

  usb_host_config_t host_config = {
    .skip_phy_setup = false,
    .intr_flags = ESP_INTR_FLAG_LEVEL1,
  };

  if (usb_host_install(&host_config) == ESP_OK) {
    ESP_LOGI(TAG, "USB Host installed");
    xSemaphoreGive(sem_host_ready);
  } else {
    ESP_LOGE(TAG, "USB Host install failed");
    vTaskDelete(NULL);
    return;
  }

  while (1) {
    uint32_t event_flags;
    usb_host_lib_handle_events(portMAX_DELAY, &event_flags);

    if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
      ESP_LOGI(TAG, "No clients connected");
    }
    if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
      ESP_LOGI(TAG, "All devices freed");
    }
  }
}

static void client_event_cb(const usb_host_client_event_msg_t *event_msg, void *arg) {
  class_driver_ctrl_t *obj = (class_driver_ctrl_t *)arg;
  switch (event_msg->event) {
    case USB_HOST_CLIENT_EVENT_NEW_DEV:
      obj->actions |= CLASS_DRIVER_ACTION_OPEN_DEV;
      obj->dev_addr = event_msg->new_dev.address;
      break;
    case USB_HOST_CLIENT_EVENT_DEV_GONE:
      obj->actions |= CLASS_DRIVER_ACTION_CLOSE_DEV;
      break;
    default:
      break;
  }
}

void client_task(void* pvParameters) {
  RCS300* self = static_cast<RCS300*>(pvParameters);

  xSemaphoreTake(sem_host_ready, portMAX_DELAY);

  usb_host_client_config_t client_config = {
    .is_synchronous = false,
    .max_num_event_msg = 5,
    .async = {
      .client_event_callback = client_event_cb,
      .callback_arg = &self->ctrl,
    }
  };
  usb_host_client_register(&client_config, &self->ctrl.client_hdl);

  while (1) {
    usb_host_client_handle_events(self->ctrl.client_hdl, portMAX_DELAY);

    if (self->ctrl.actions & CLASS_DRIVER_ACTION_OPEN_DEV) {
      usb_host_device_open(self->ctrl.client_hdl, self->ctrl.dev_addr, &self->ctrl.dev_hdl);
      const usb_device_desc_t *dev_desc;
      usb_host_get_device_descriptor(self->ctrl.dev_hdl, &dev_desc);
      if (dev_desc->idVendor == RCS300_VID && dev_desc->idProduct == RCS300_PID) {
        ESP_LOGI(TAG, "RC-S300 Connected");
        self->connected = true;
        usb_host_interface_claim(self->ctrl.client_hdl, self->ctrl.dev_hdl, 1, 0);
        self->ctrl.actions &= ~CLASS_DRIVER_ACTION_OPEN_DEV;
      } else {
        self->ctrl.actions |= CLASS_DRIVER_ACTION_CLOSE_DEV;
      }
    }

    if (self->ctrl.actions & CLASS_DRIVER_ACTION_CLOSE_DEV) {
      self->connected = false;
      usb_host_interface_release(self->ctrl.client_hdl, self->ctrl.dev_hdl, 1);
      usb_host_device_close(self->ctrl.client_hdl, self->ctrl.dev_hdl);
      self->ctrl.dev_hdl = NULL;
      self->ctrl.actions &= ~CLASS_DRIVER_ACTION_CLOSE_DEV;
    }
  }
}

static void transfer_done_cb(usb_transfer_t *transfer) {
  usb_xfer_context_t *ctx = (usb_xfer_context_t *)transfer->context;

  if (transfer->status == USB_TRANSFER_STATUS_COMPLETED) {
    ctx->result = ESP_OK;
    if (transfer->bEndpointAddress & 0x80) {
      ctx->resp_len = transfer->actual_num_bytes;
      if (ctx->resp) {
        memcpy(ctx->resp, transfer->data_buffer, ctx->resp_len);
      }
    }
  } else {
    ctx->result = ESP_FAIL;
  }
  xSemaphoreGive(ctx->sem);
}

RCS300::RCS300() {
  connected = false;
  req_ptr = &req;
  ctrl = {0};
}

esp_err_t RCS300::usb_execute_command(const uint8_t *cmd, size_t cmd_len, uint8_t *resp, size_t *resp_len) {
  if (cmd_len > CMD_BUF_MAX) {
    return ESP_ERR_INVALID_SIZE;
  }

  memcpy(req_ptr->cmd, cmd, cmd_len);
  req_ptr->cmd_len = cmd_len;
  req_ptr->resp_len = 0;
  req_ptr->result = usb_transfer(ctrl.client_hdl, ctrl.dev_hdl);

  ESP_LOGI(TAG, "Received: %d bytes", req_ptr->resp_len);

  if (resp && resp_len) {
    memcpy(resp, req_ptr->resp, req_ptr->resp_len);
    *resp_len = req_ptr->resp_len;
  }

  return req_ptr->result;
}

esp_err_t RCS300::polling(uint16_t systemCode) {
  uint8_t resp[RESP_BUF_MAX];
  size_t resp_len;

  // endTransparent
  const uint8_t cmd1[] = {0x6B, 0x08, 0x00, 0x00, 0x00, 0x00, 0x21, 0x00, 0x00, 0x00, 0xFF, 0x50, 0x00, 0x00, 0x02, 0x82, 0x00, 0x00};
  ESP_LOGI(TAG, "cmd1 send");
  esp_err_t ret1 = usb_execute_command(cmd1, sizeof(cmd1), resp, &resp_len);
  if (ret1 != ESP_OK) {
    return ret1;
  } 
  ESP_LOGI(TAG, "cmd1 respond: %d", resp_len);
  if (resp[resp_len - 2] != 0x90 || resp[resp_len - 1] != 0x00) {
    return ESP_ERR_INVALID_RESPONSE;
  }

  // startTransparent
  const uint8_t cmd2[] = {0x6B, 0x08, 0x00, 0x00, 0x00, 0x00, 0x21, 0x00, 0x00, 0x00, 0xFF, 0x50, 0x00, 0x00, 0x02, 0x81, 0x00, 0x00};
  ESP_LOGI(TAG, "cmd2 send");
  esp_err_t ret2 = usb_execute_command(cmd2, sizeof(cmd2), resp, &resp_len);
  if (ret2 != ESP_OK) {
    return ret2;
  }
  ESP_LOGI(TAG, "cmd2 respond: %d", resp_len);
  if (resp[resp_len - 2] != 0x90 || resp[resp_len - 1] != 0x00) {
    return ESP_ERR_INVALID_RESPONSE;
  }

  // rfOff
  const uint8_t cmd3[] = {0x6B, 0x08, 0x00, 0x00, 0x00, 0x00, 0x21, 0x00, 0x00, 0x00, 0xFF, 0x50, 0x00, 0x00, 0x02, 0x83, 0x00, 0x00};
  ESP_LOGI(TAG, "cmd3 send");
  esp_err_t ret3 = usb_execute_command(cmd3, sizeof(cmd3), resp, &resp_len);
  if (ret3 != ESP_OK) {
    return ret3;
  }
  ESP_LOGI(TAG, "cmd3 respond: %d", resp_len);
  if (resp[resp_len - 2] != 0x90 || resp[resp_len - 1] != 0x00) {
    return ESP_ERR_INVALID_RESPONSE;
  }

  // rfOn
  const uint8_t cmd4[] = {0x6B, 0x08, 0x00, 0x00, 0x00, 0x00, 0x21, 0x00, 0x00, 0x00, 0xFF, 0x50, 0x00, 0x00, 0x02, 0x84, 0x00, 0x00};
  ESP_LOGI(TAG, "cmd4 send");
  esp_err_t ret4 = usb_execute_command(cmd4, sizeof(cmd4), resp, &resp_len);
  if (ret4 != ESP_OK) {
    return ret4;
  }
  ESP_LOGI(TAG, "cmd4 respond: %d", resp_len);
  if (resp[resp_len - 2] != 0x90 || resp[resp_len - 1] != 0x00) {
    return ESP_ERR_INVALID_RESPONSE;
  }

  // switchProtcolTypeF
  const uint8_t cmd5[] = {0x6B, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x2B, 0x00, 0x00, 0x00, 0xFF, 0x50, 0x00, 0x02, 0x04, 0x8F, 0x02, 0x03, 0x00, 0x00};
  ESP_LOGI(TAG, "cmd5 send");
  esp_err_t ret5 = usb_execute_command(cmd5, sizeof(cmd5), resp, &resp_len);
  if (ret5 != ESP_OK) {
    return ret5;
  }
  ESP_LOGI(TAG, "cmd5 respond: %d", resp_len);
  if (resp[resp_len - 2] != 0x90 || resp[resp_len - 1] != 0x00) {
    return ESP_ERR_INVALID_RESPONSE;
  }

  // polling
  const uint8_t cmd6[] = {0x6B, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x2C, 0x00, 0x00, 0x00, 0xFF, 0x50, 0x00, 0x01, 0x00, 0x00, 0x11,
    0x5F, 0x46, 0x04, 0xA0, 0x86, 0x01, 0x00, 0x95, 0x82, 0x00, 0x06, 0x06, 0x00, (uint8_t)(systemCode>>8), (uint8_t)systemCode, 0x01, 0x00, 0x00, 0x00, 0x00};
  ESP_LOGI(TAG, "cmd6 send");
  esp_err_t ret6 = usb_execute_command(cmd6, sizeof(cmd6), resp, &resp_len);
  if (ret6 != ESP_OK) {
    return ret6;
  }
  ESP_LOGI(TAG, "cmd6 respond: %d", resp_len);
  if (resp[resp_len - 2] != 0x90 || resp[resp_len - 1] != 0x00) {
    return ESP_ERR_INVALID_RESPONSE;
  }
  if (resp_len == 46) {
    memcpy(idm, resp + 26, 8);
    memcpy(pmm, resp + 34, 8);
    return ESP_OK;
  }
  return ESP_ERR_NOT_FOUND;
}

void RCS300::begin(void) {
  sem_host_ready = xSemaphoreCreateBinary();
  xTaskCreatePinnedToCore(daemon_task, "daemon_task", 4096, this, 1, NULL, 0);
  xTaskCreatePinnedToCore(client_task, "client_task", 4096, this, 1, NULL, 0);
}

esp_err_t RCS300::usb_transfer(usb_host_client_handle_t client_hdl, usb_device_handle_t dev_hdl) {
  usb_xfer_context_t ctx = {
    .resp = req_ptr->resp,
    .resp_len = 0,
    .sem = xSemaphoreCreateBinary(),
    .result = ESP_FAIL
  };
  if (!ctx.sem) return ESP_ERR_NO_MEM;

  usb_transfer_t *xfer_out;
  usb_transfer_t *xfer_in;

  // OUT
  usb_host_transfer_alloc(req_ptr->cmd_len, 0, &xfer_out);
  memcpy(xfer_out->data_buffer, req_ptr->cmd, req_ptr->cmd_len);
  xfer_out->num_bytes = req_ptr->cmd_len;
  xfer_out->device_handle = dev_hdl;
  xfer_out->bEndpointAddress = RCS300_EP;
  xfer_out->callback = transfer_done_cb;
  xfer_out->context = &ctx;

  esp_err_t ret1 = usb_host_transfer_submit(xfer_out);
  if (ret1 != ESP_OK) {
    vSemaphoreDelete(ctx.sem);
    return ret1;
  }
  xSemaphoreTake(ctx.sem, portMAX_DELAY);
  usb_host_transfer_free(xfer_out);
  if (ctx.result != ESP_OK) {
    vSemaphoreDelete(ctx.sem);
    return ctx.result;
  }

  // IN
  usb_host_transfer_alloc(RESP_BUF_MAX, 0, &xfer_in);
  xfer_in->num_bytes = RESP_BUF_MAX;
  xfer_in->device_handle = dev_hdl;
  xfer_in->bEndpointAddress = RCS300_EP | 0x80;
  xfer_in->callback = transfer_done_cb;
  xfer_in->context = &ctx;

  esp_err_t ret2 = usb_host_transfer_submit(xfer_in);
  if (ret2 != ESP_OK) {
    vSemaphoreDelete(ctx.sem);
    return ret2;
  }
  xSemaphoreTake(ctx.sem, portMAX_DELAY);
  req_ptr->resp_len = ctx.resp_len;
  usb_host_transfer_free(xfer_in);

  vSemaphoreDelete(ctx.sem);
  return ctx.result;
}
