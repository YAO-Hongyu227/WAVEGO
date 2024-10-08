#include <arduino.h>
#include "New_WebPage.h"
// Replace with your network credentials
// WIFI_AP settings.
// const char* AP_SSID = "看h片上91搜 192.168.4.1";     //dog3
const char* AP_SSID = "dog3看h片上91搜 192.168.4.1";     //dog1
const char* AP_PWD  = "1234567890";

// WIFI_STA settings.
const char* STA_SSID = "OnePlus 8";
const char* STA_PWD  = "40963840";

// set the default wifi mode here.
// 1 as [AP] mode, it will not connect other wifi.
// also, 1 as upper computer control mode.
// 2 as [STA] mode, it will connect to know wifi.
#define DEFAULT_WIFI_MODE 1
extern int WIFIP_MODE;

extern IPAddress IP_ADDRESS;
extern byte WIFI_MODE;
extern String MAC_ADDRESS;
extern int WIFI_RSSI;

#include "esp_wifi.h"
#include <WiFi.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

#include "dl_lib_matrix3d.h"
#include <esp32-hal-ledc.h>
#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "img_converters.h"

#define CAMERA_MODEL_ESP_EYE
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     4
#define SIOD_GPIO_NUM     18
#define SIOC_GPIO_NUM     23

#define Y9_GPIO_NUM       36
#define Y8_GPIO_NUM       37
#define Y7_GPIO_NUM       38
#define Y6_GPIO_NUM       39
#define Y5_GPIO_NUM       35
#define Y4_GPIO_NUM       14
#define Y3_GPIO_NUM       13
#define Y2_GPIO_NUM       34
#define VSYNC_GPIO_NUM    5
#define HREF_GPIO_NUM     27
#define PCLK_GPIO_NUM     25

extern int MiddlePosition;
extern int moveFB;
extern int moveLR;
extern int debugMode;
extern int funcMode;
extern void initPosAll();
extern void middlePosAll();
extern void servoDebug(byte servoID, int offset);
extern void servoConfigSave(byte activeServo);
extern int ServoMiddlePWM[16];
extern int CurrentPWM[16];

// BUZZER INIT.
#define BUZZER  21

// YAO, 7/6/2024
extern double COM_FB;
extern double COM_LR;
extern double PID_X;
extern double PID_Z;
extern double WALK_HEIGHT;
extern float target_roll;
extern float target_pitch;


extern bool upper_flag;
extern float turning_direction;
extern float walking_distance;
extern int crab_gait; 
extern float adjust_yaw;
extern double global_yaw;
extern bool NOTWALK;

extern double WALK_LIFT;
extern double WALK_HEIGHT;
extern int   STEP_DELAY;       //ms
extern float STEP_ITERATE;
extern double WALK_MASS_ADJUST;
extern float turning_direction;

extern bool imu_on;
extern int STEP_DELAY;
extern float STEP_ITERATE;

extern float Once_Pitch_threshold;   //10.0
extern float Once_Roll_threshold;     //5.0
extern double Adjust_Pitch_threshold;   //10.0
extern double Adjust_Roll_threshold;     //1.0
extern float kp_pitch;  //0.01
extern float kp_roll;   //0.005

extern double right_leg_bias;

//triangular gait
extern float offset_x; 
extern float offset_z;
extern float offset_cycleinput; 
extern float RLB_bias;

// Buzzer
bool isbuzz = false;
float buz_freq = 1;
//END



extern void getMAC(){
  WiFi.mode(WIFI_MODE_STA);
  MAC_ADDRESS = WiFi.macAddress();
  // Serial.print("MAC:");
  // Serial.println(WiFi.macAddress());
}


extern void getIP(){
  IP_ADDRESS = WiFi.localIP();
}


void setAP(){
  WiFi.softAP(AP_SSID, AP_PWD);
  IPAddress myIP = WiFi.softAPIP();
  IP_ADDRESS = myIP;
  // Serial.print("AP IP address: ");
  // Serial.println(myIP);
  WIFI_MODE = 1;
}


void setSTA(){
  WIFI_MODE = 3;
  WiFi.begin(STA_SSID, STA_PWD);
}


extern void getWifiStatus(){
  if(WiFi.status() == WL_CONNECTED){
    WIFI_MODE = 2;
    getIP();
    WIFI_RSSI = WiFi.RSSI();
  }
  else if(WiFi.status() == WL_CONNECTION_LOST && DEFAULT_WIFI_MODE == 2){
    WIFI_MODE = 3;
    // WiFi.disconnect();
    WiFi.reconnect();
  }
}


void wifiInit(){
  WIFI_MODE = DEFAULT_WIFI_MODE;
  if(WIFI_MODE == 1){setAP();}
  else if(WIFI_MODE == 2){setSTA();}
}


typedef struct {
  httpd_req_t *req;
  size_t len;
} jpg_chunking_t;
 
#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";
 
httpd_handle_t stream_httpd = NULL;
httpd_handle_t camera_httpd = NULL;

static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t * fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t * _jpg_buf = NULL;
  char * part_buf[64];
  dl_matrix3du_t *image_matrix = NULL;
 
  static int64_t last_frame = 0;
  if (!last_frame) {
    last_frame = esp_timer_get_time();
  }
 
  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if (res != ESP_OK) {
    return res;
  }
 
  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) {
      // Serial.println("Camera capture failed");
      res = ESP_FAIL;
    } 
    else {
      {
        if (fb->format != PIXFORMAT_JPEG) {
          bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
          esp_camera_fb_return(fb);
          fb = NULL;
          if (!jpeg_converted) {
            // Serial.println("JPEG compression failed");
            res = ESP_FAIL;
          }
        } else {
          _jpg_buf_len = fb->len;
          _jpg_buf = fb->buf;
        }
      }
    }
    if(res == ESP_OK){
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    }
    if (res == ESP_OK) {
      size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
      res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
    }
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    }

    if (fb) {
      esp_camera_fb_return(fb);
      fb = NULL;
      _jpg_buf = NULL;
    } else if (_jpg_buf) {
      free(_jpg_buf);
      _jpg_buf = NULL;
    }
    if (res != ESP_OK) {
      break;
    }
    int64_t fr_end = esp_timer_get_time();
    int64_t frame_time = fr_end - last_frame;
    last_frame = fr_end;
    frame_time /= 1000;

    delay(10);
  }
 
  last_frame = 0;
  return res;
}

 
static esp_err_t cmd_handler(httpd_req_t *req){
  char*  buf;
  size_t buf_len;
  char variable[32] = {0,};
  char value[32] = {0,};
  char cmd[32] = {0,};
 
  buf_len = httpd_req_get_url_query_len(req) + 1;
  if (buf_len > 1) {
    buf = (char*)malloc(buf_len);
    if (!buf) {
      httpd_resp_send_500(req);
      return ESP_FAIL;
    }
    if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
      if (httpd_query_key_value(buf, "var", variable, sizeof(variable)) == ESP_OK &&
          httpd_query_key_value(buf, "val", value, sizeof(value)) == ESP_OK &&
          httpd_query_key_value(buf, "cmd", cmd, sizeof(cmd)) == ESP_OK) {
      } else {
        free(buf);
        httpd_resp_send_404(req);
        return ESP_FAIL;
      }
    } else {
      free(buf);
      httpd_resp_send_404(req);
      return ESP_FAIL;
    }
    free(buf);
  } else {
    httpd_resp_send_404(req);
    return ESP_FAIL;
  }
 
  int val = atoi(value);
  int cmdint = atoi(cmd);
  sensor_t * s = esp_camera_sensor_get();
  int res = 0;
 
  // Look at values within URL to determine function
  if (!strcmp(variable, "framesize")){
    Serial.println("framesize");
    if (s->pixformat == PIXFORMAT_JPEG) res = s->set_framesize(s, (framesize_t)val);
  }

  // functions ctrl.
  else if (!strcmp(variable, "funcMode")){
    debugMode = 0;
    if (val == 1){
      if(funcMode == 1){funcMode = 0;Serial.println("Steady OFF");}
      else if(funcMode == 0){funcMode = 1;Serial.println("Steady ON");}
    }
    else{
      funcMode = val;
      Serial.println(val);
    }        
  }

  // servo config. debugMode
  // val as servoID, cmdint as command.
  else if (!strcmp(variable, "sconfig")){
    debugMode = 1;
    funcMode = 0;
    servoDebug(val, cmdint);
    Serial.print("servo:");Serial.print(val);Serial.print(" position:");Serial.print(CurrentPWM[val]);
    Serial.print(" MID:");Serial.print(ServoMiddlePWM[val]);Serial.print(" offset:");Serial.println(cmdint);
  }
  else if (!strcmp(variable, "sset")){
    if(debugMode){
    servoConfigSave(val);
    Serial.print("SET servo:");Serial.print(val);Serial.print(" position:");Serial.println(ServoMiddlePWM[val]);
    }
    else{
      Serial.print("DebugMode = 0, servo config could not be saved.");
    }
  }

  // move ctrl.
  else if (!strcmp(variable, "move")){
    debugMode = 0;
    funcMode  = 0;
    if (val == 1) {
      Serial.println("Forward");
      moveFB = 1;
    }
    else if (val == 2) {
      Serial.println("TurnLeft");
      moveLR = -1;
    }
    else if (val == 3) {
      Serial.println("FBStop");
      moveFB = 0;
    }
    else if (val == 4) {
      Serial.println("TurnRight");
      moveLR = 1;
    }
    else if (val == 5) {
      Serial.println("Backward");
      moveFB = -1;
    }
    else if (val == 6){
      Serial.println("LRStop");
      moveLR = 0;
    }
  }


// debugging walking gait parameters

///////////////////////////////////////////////////////////////////////////////// FB COM /////
   else if (!strcmp(variable, "AdjustFBcom")){
    debugMode = 0;
    COM_FB += val;
    Serial.print("COM_FB: ");
    Serial.println(COM_FB);       
  }
  else if (!strcmp(variable, "DiceAdjustFBcom")){
    debugMode = 0;
    COM_FB += val*0.1;
    Serial.print("COM_FB: ");
    Serial.println(COM_FB);       
  }


///////////////////////////////////////////////////////////////////////////////// LR COM /////

   else if (!strcmp(variable, "AdjustLRcom")){
    debugMode = 0;
    COM_LR += val;
    Serial.print("COM_LR: ");
    Serial.println(COM_LR);       
  }
  else if (!strcmp(variable, "DiceAdjustLRcom")){
    debugMode = 0;
    COM_LR += val*0.1;
    Serial.print("COM_LR: ");
    Serial.println(COM_LR);       
  }

///////////////////////////////////////////////////////////////////////////////// FREE TROT /////
   else if (!strcmp(variable, "freetrot")){
    debugMode = 0;
    int dis = val;
    upper_flag = true;
    double d_dis = dis;
    walking_distance = d_dis;
    //turning_direction = 0;
    NOTWALK = false;
    // funcMode = 12; // trot gait
    funcMode = 19;  //triangular gait
  }

///////////////////////////////////////////////////////////////////////////////// Walk direction /////
   else if (!strcmp(variable, "walkdirection")){
    debugMode = 0;
    upper_flag = true;
    double d_val = val;
    turning_direction = d_val;
    NOTWALK = false;
  }


///////////////////////////////////////////////////////////////////////////////// FREE TROT /////
   else if (!strcmp(variable, "trotdirection")){
    debugMode = 0;
    upper_flag = true;
    double d_val = val;
    turning_direction = d_val;
    NOTWALK = false;
    funcMode = 17;
  }
///////////////////////////////////////////////////////////////////////////////// WALK LIFT /////
   else if (!strcmp(variable, "walkheight")){
    debugMode = 0;
    int dis = val;
    double d_dis = dis;
    WALK_LIFT = d_dis;
    Serial.print(" WALK_LIFT: ");
    Serial.println(WALK_LIFT);
  }


///////////////////////////////////////////////////////////////////////////////// WALK HEIGHT /////
   else if (!strcmp(variable, "standheight")){
    debugMode = 0;
    int dis = val;
    double d_dis = dis;
    WALK_HEIGHT = d_dis;
    Serial.print(" WALK_HEIGHT: ");
    Serial.println(WALK_HEIGHT);
  }

///////////////////////////////////////////////////////////////////////////////// WALK HEIGHT /////
   else if (!strcmp(variable, "rightbias")){
    debugMode = 0;
    int bias = val;
    double d_bias = bias;
    right_leg_bias += d_bias*0.01;
    Serial.print(" Right Leg Bias: ");
    Serial.println(right_leg_bias);
  }

///////////////////////////////////////////////////////////////////////////////// IMU SWITCH /////
   else if (!strcmp(variable, "imuswitch")){
    debugMode = 0;
    if (val == 1)
    {
      imu_on = true;
      Serial.println("IMU turned ON");
    }
    else
    {
      imu_on = false;
      Serial.println("IMU turned OFF");
    }
  }

///////////////////////////////////////////////////////////////////////////////// KP /////
   else if (!strcmp(variable, "kppitch")){
    debugMode = 0;
    kp_pitch = val * 0.001;
    Serial.print("kp pitch: ");
    Serial.println(kp_pitch);
  }

   else if (!strcmp(variable, "kproll")){
    debugMode = 0;
    kp_roll = val * 0.001;
    Serial.print("kp roll: ");
    Serial.println(kp_roll);
  }


///////////////////////////////////////////////////////////////////////////////// STEP_DELAY /////

   else if (!strcmp(variable, "stepdelay")){
    debugMode = 0;
    STEP_DELAY += val;
    Serial.print("STEP DELAY: ");
    Serial.println(STEP_DELAY);
  }

///////////////////////////////////////////////////////////////////////////////// STEP_ITERATE /////

   else if (!strcmp(variable, "stepiterate")){
    debugMode = 0;
    STEP_ITERATE += val*0.01;
    Serial.print("STEP ITERATE: ");
    Serial.println(STEP_ITERATE);
  }

///////////////////////////////////////////////////////////////////////////////// OFFSET_X /////
else if (!strcmp(variable, "offsetx")){
    debugMode = 0;
    offset_x += val*0.1;
    Serial.print("OFFSET X: ");
    Serial.println(offset_x);
  }

///////////////////////////////////////////////////////////////////////////////// OFFSET_Z /////
else if (!strcmp(variable, "offsetz")){
    debugMode = 0;
    offset_z += val*0.1;
    Serial.print("OFFSET Z: ");
    Serial.println(offset_z);
  }
///////////////////////////////////////////////////////////////////////////////// OFFSET CYCLE /////
else if (!strcmp(variable, "offsetcycle")){
    debugMode = 0;
    offset_cycleinput = val*0.001;
    Serial.print("OFFSET CYCLEINPUT: ");
    Serial.println(offset_cycleinput);
  }
///////////////////////////////////////////////////////////////////////////////// RLB Bias /////
else if (!strcmp(variable, "rlbbias")){
    debugMode = 0;
    RLB_bias += val*0.01;
    Serial.print("RLB Bias: ");
    Serial.println(RLB_bias);
  }
///////////////////////////////////////////////////////////////////////////////// Threshold /////
   else if (!strcmp(variable, "OPT")){
    debugMode = 0;
    Once_Pitch_threshold += val;
    Serial.print("Once_Pitch_threshold: ");
    Serial.println(Once_Pitch_threshold);
  }

   else if (!strcmp(variable, "APT")){
    debugMode = 0;
    Adjust_Pitch_threshold += val;
    Serial.print("Adjust_Pitch_threshold: ");
    Serial.println(Adjust_Pitch_threshold);
  }

   else if (!strcmp(variable, "ORT")){
    debugMode = 0;
    Once_Roll_threshold += val;
    Serial.print("Once_Roll_threshold: ");
    Serial.println(Once_Roll_threshold);
  }

   else if (!strcmp(variable, "ART")){
    debugMode = 0;
    Adjust_Roll_threshold += val;
    Serial.print("Adjust_Roll_threshold: ");
    Serial.println(Adjust_Roll_threshold);
  }

// debugging walking gait parameters END
///////////////////////////////////////////////////////////////////////////// BUZZER
   else if (!strcmp(variable, "buzzerfreq")){
    debugMode = 0;
    buz_freq = val;
    Serial.print("Buzzer Frequency: ");
    Serial.println(buz_freq);
  }
  else if (!strcmp(variable, "freeclimb")){
    debugMode = 0;
    upper_flag = true;
    double d_val = val;
    turning_direction = d_val;
    NOTWALK = false;
    funcMode = 16;
    Serial.print("Free Climb: ");
    Serial.println(turning_direction);
  }

else if (!strcmp(variable, "buzzeron")) {
    debugMode = 0;
    unsigned long startMillis = millis(); 
    unsigned long currentMillis = startMillis;
    unsigned long duration = val*1000; 
    float delayTime = float(1) / buz_freq * 1000; 

    while (currentMillis - startMillis < duration) {
        digitalWrite(BUZZER, LOW);
        delay(delayTime / 2); 
        digitalWrite(BUZZER, HIGH);
        delay(delayTime / 2);
        currentMillis = millis();
    }
    Serial.println("BUZZER ON for 3 seconds");
}



  else
  {
    Serial.println("variable");
    res = -1;
  }
 
  if (res) {
    return httpd_resp_send_500(req);
  }
 
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, NULL, 0);
}

 
static esp_err_t index_handler(httpd_req_t *req){
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, (const char *)INDEX_HTML, strlen(INDEX_HTML));
}


void startCameraServer(){
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
 
    httpd_uri_t index_uri = {
        .uri       = "/",
        .method    = HTTP_GET,
        .handler   = index_handler,
        .user_ctx  = NULL
    };
 
    httpd_uri_t cmd_uri = {
        .uri       = "/control",
        .method    = HTTP_GET,
        .handler   = cmd_handler,
        .user_ctx  = NULL
    };
 
   httpd_uri_t stream_uri = {
        .uri       = "/stream",
        .method    = HTTP_GET,
        .handler   = stream_handler,
        .user_ctx  = NULL
    };
    
    // Serial.printf("Starting web server on port: '%d'\n", config.server_port);
    if (httpd_start(&camera_httpd, &config) == ESP_OK) {
        httpd_register_uri_handler(camera_httpd, &index_uri);
        httpd_register_uri_handler(camera_httpd, &cmd_uri);
    }
 
    config.server_port += 1;
    config.ctrl_port += 1;
    // Serial.printf("Starting stream server on port: '%d'\n", config.server_port);
    if (httpd_start(&stream_httpd, &config) == ESP_OK) {
        httpd_register_uri_handler(stream_httpd, &stream_uri);
    }
}


void webServerInit(){
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // prevent brownouts by silencing them

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;

  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;

  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  //init with high specs to pre-allocate larger buffers
  config.frame_size = FRAMESIZE_QVGA;
  config.jpeg_quality = 63;
  config.fb_count = 1;

  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    // Serial.printf("Camera init failed with error 0x%x", err);
    // return;
  }
  else{
    sensor_t * s = esp_camera_sensor_get();
    s->set_saturation(s, 2);
    delay(1000); 
  }

  wifiInit();
  delay(1000); 
  startCameraServer();
}
