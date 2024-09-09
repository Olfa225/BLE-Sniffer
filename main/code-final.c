#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>  // Pour utiliser PRIu32
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "driver/i2c.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "driver/adc.h"
#include "driver/ledc.h"

static const char *TAG = "BLE_Device";

#define SERVO_GPIO_PIN 18
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_DUTY_RES LEDC_TIMER_14_BIT // Set duty resolution to 14 bits
#define LEDC_FREQUENCY 50               // Frequency in Hertz
#define SERVO_MIN_PULSEWIDTH 500        // Min largeur d'impulsion en microsecondes
#define SERVO_MAX_PULSEWIDTH 2500       // Max largeur d'impulsion en microsecondes
#define SERVO_MAX_DEGREE 180            // Angle maximal en degrés

#define PULSE_PIN ADC1_CHANNEL_0  // Correspond à GPIO0
#define THRESHOLD 100
#define I2C_MASTER_SCL_IO 9  // GPIO utilisé pour SCL
#define I2C_MASTER_SDA_IO 8  // GPIO utilisé pour SDA
#define I2C_MASTER_NUM I2C_NUM_0  // Numéro du port I2C
#define I2C_MASTER_FREQ_HZ 400000  // Fréquence du bus I2C
#define I2C_MASTER_TX_BUF_DISABLE 0  // Désactivation du buffer TX
#define I2C_MASTER_RX_BUF_DISABLE 0  // Désactivation du buffer RX
#define I2C_MASTER_TIMEOUT_MS 1000  // Timeout en millisecondes
#define SLAVE_ADDRESS_LCD 0x27  // Adresse I2C de l'écran LCD


static int current_heart_rate = 0;
uint8_t ble_addr_type;

// UUIDs pour les services
static const ble_uuid16_t heart_rate_service_uuid = BLE_UUID16_INIT(0x180D);  // Heart Rate Service
static const ble_uuid128_t service_uuid = BLE_UUID128_INIT(0x12, 0x34, 0x56, 0x78, 0x12, 0x34, 0x56, 0x78, 0x12, 0x34, 0x56, 0x78, 0x90, 0xAB, 0xCD, 0xEF);

// UUIDs pour les caractéristiques
 // Heart Rate Measurement
static const ble_uuid128_t servo_char_uuid = BLE_UUID128_INIT(0xAB, 0xCD, 0xEF, 0x01, 0x23, 0x45, 0x67, 0x89, 0x01, 0x23, 0x45, 0x67, 0x89, 0x01, 0x02);
static const ble_uuid128_t characteristic_uuid = BLE_UUID128_INIT(0xAB, 0xCD, 0xEF, 0x01, 0x23, 0x45, 0x67, 0x89, 0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF);


void ble_app_advertise(void);  // Déclarer la fonction avant son utilisation

// Définition de la fonction angle_to_duty_cycle
static uint32_t angle_to_duty_cycle(uint32_t angle) {
    uint32_t pulsewidth = (SERVO_MIN_PULSEWIDTH + ((SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) * angle) / SERVO_MAX_DEGREE);
    uint32_t max_duty = (1 << LEDC_DUTY_RES) - 1;
    return (pulsewidth * max_duty) / 20000; // Convertir la largeur d'impulsion en duty cycle
}

// Initialisation de l'I2C pour l'écran LCD
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(i2c_master_port, &conf));
    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

// Fonction pour envoyer une commande à l'écran LCD
void lcd_send_cmd(char cmd)
{
    esp_err_t err;
    char data_u, data_l;
    uint8_t data_t[4];
    data_u = (cmd & 0xF0);
    data_l = ((cmd << 4) & 0xF0);
    data_t[0] = data_u | 0x0C; // en=1, rs=0
    data_t[1] = data_u | 0x08; // en=0, rs=0
    data_t[2] = data_l | 0x0C; // en=1, rs=0
    data_t[3] = data_l | 0x08; // en=0, rs=0
    err = i2c_master_write_to_device(I2C_MASTER_NUM, SLAVE_ADDRESS_LCD, data_t, 4, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    if (err != ESP_OK) ESP_LOGI(TAG, "Erreur lors de l'envoi de la commande: %d", err);
}

// Fonction pour envoyer des données à l'écran LCD
void lcd_send_data(char data)
{
    esp_err_t err;
    char data_u, data_l;
    uint8_t data_t[4];
    data_u = (data & 0xF0);
    data_l = ((data << 4) & 0xF0);
    data_t[0] = data_u | 0x0D; // en=1, rs=1
    data_t[1] = data_u | 0x09; // en=0, rs=1
    data_t[2] = data_l | 0x0D; // en=1, rs=1
    data_t[3] = data_l | 0x09; // en=0, rs=1
    err = i2c_master_write_to_device(I2C_MASTER_NUM, SLAVE_ADDRESS_LCD, data_t, 4, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    if (err != ESP_OK) ESP_LOGI(TAG, "Erreur lors de l'envoi des données: %d", err);
}

// Fonction pour initialiser l'écran LCD
void lcd_init(void)
{
    vTaskDelay(50 / portTICK_PERIOD_MS); // attente >40ms
    lcd_send_cmd(0x30);
    vTaskDelay(5 / portTICK_PERIOD_MS);  // attente >4.1ms
    lcd_send_cmd(0x30);
    vTaskDelay(1 / portTICK_PERIOD_MS);   // attente >100us
    lcd_send_cmd(0x30);
    vTaskDelay(1 / portTICK_PERIOD_MS);
    lcd_send_cmd(0x20);  // mode 4 bits
    vTaskDelay(1 / portTICK_PERIOD_MS);

    // initialisation de l'affichage
    lcd_send_cmd(0x28); // fonction set: DL=0 (mode 4 bits), N=1 (2 lignes), F=0 (5x8 caractères)
    vTaskDelay(1 / portTICK_PERIOD_MS);
    lcd_send_cmd(0x08); // affichage on/off: D=0, C=0, B=0 -> affichage off
    vTaskDelay(1 / portTICK_PERIOD_MS);
    lcd_send_cmd(0x01);  // clear display
    vTaskDelay(5 / portTICK_PERIOD_MS);
    lcd_send_cmd(0x06); // entry mode set: I/D=1 (curseur incrémente) & S=0 (pas de shift)
    vTaskDelay(1 / portTICK_PERIOD_MS);
    lcd_send_cmd(0x0C); // affichage on/off: D=1, C=0, B=0 -> affichage on, curseur off
    vTaskDelay(1 / portTICK_PERIOD_MS);
}

// Fonction pour positionner le curseur sur l'écran LCD
void lcd_set_cursor(uint8_t col, uint8_t row)
{
    uint8_t row_offsets[] = {0x00, 0x40, 0x14, 0x54};
    lcd_send_cmd(0x80 | (col + row_offsets[row]));
}

// Fonction pour effacer une ligne sur l'écran LCD
void lcd_clear_line(uint8_t row)
{
    lcd_set_cursor(0, row);
    for (int i = 0; i < 16; i++)
    {
        lcd_send_data(' ');
    }
    lcd_set_cursor(0, row);
}

// Fonction pour afficher une chaîne de caractères sur l'écran LCD
void lcd_send_string(char *str)
{
    while (*str) lcd_send_data(*str++);
}

// Fonction pour commander le servo moteur via BLE
static int servo_write(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int angle = atoi((char *)ctxt->om->om_data);
    if (angle <= 180) {  // Vérification de l'angle
        uint32_t duty = angle_to_duty_cycle(angle);
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
        ESP_LOGI(TAG, "Servo angle set to %d degrees", angle);
    } else {
        ESP_LOGW(TAG, "Invalid servo angle: %d", angle);
    }
    return 0;
}
// Fonction pour lire la fréquence cardiaque
int read_heart_rate_sensor() {
    int raw_value = adc1_get_raw(PULSE_PIN);
    int heart_rate = (raw_value * 200) / 4095; // Conversion en BPM
    return heart_rate;


  while (1){
 
        int heart_rate = read_heart_rate_sensor();

        // Afficher la valeur du BPM sur le moniteur série
        ESP_LOGI(TAG, "Fréquence cardiaque : %d BPM", heart_rate);
         // Pause de 500 ms
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void update_heart_rate_display(int heart_rate) {
    // Effacer la deuxième ligne
    lcd_clear_line(1);

    // Placer le curseur sur la deuxième ligne
    lcd_set_cursor(0, 1);

    // Afficher la valeur du BPM sur l'écran LCD
    char bpm_str[16];
    snprintf(bpm_str, sizeof(bpm_str), "BPM: %d", heart_rate);
    lcd_send_string(bpm_str);
}


    
static int device_read(uint16_t con_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    int heart_rate = read_heart_rate_sensor();
    
    // Formatage de la valeur de la fréquence cardiaque en chaîne de caractères
    char heart_rate_str[16];
    snprintf(heart_rate_str, sizeof(heart_rate_str), "BPM: %d", heart_rate);
    
    // Affichage sur le moniteur série pour vérification
    ESP_LOGI(TAG, "Heart rate sensor value: %s", heart_rate_str);
    
    // Envoyer la chaîne de caractères via BLE
    os_mbuf_append(ctxt->om, heart_rate_str, strlen(heart_rate_str));
    
    return 0;
}



// Définition des services GATT
// Définition des services GATT
const struct ble_gatt_svc_def gatt_svcs[] = {
    // Service Heart Rate
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &heart_rate_service_uuid .u,
        .characteristics = (struct ble_gatt_chr_def[]){
            {
                .uuid = &characteristic_uuid.u,
                .flags =  BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_READ,
                .access_cb = device_read,  // Cette fonction gère les notifications
            },
            {0}
        }
    },
    // Service Servo Control
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &service_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]){
            {
                .uuid = &servo_char_uuid.u,
                .flags = BLE_GATT_CHR_F_WRITE,
                .access_cb = servo_write,  // Cette fonction gère les écritures sur la caractéristique du servo
            },
            {0}
        }
    },
    {0}
};

// Gestionnaire d'événements GAP BLE
static int ble_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type)
    {
    case BLE_GAP_EVENT_CONNECT:
        ESP_LOGI(TAG, "BLE GAP EVENT CONNECT %s", event->connect.status == 0 ? "OK!" : "FAILED!");
        if (event->connect.status != 0)
        {
            ble_app_advertise();
        }
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "BLE GAP EVENT DISCONNECTED");
        ble_app_advertise();
        break;
    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGI(TAG, "BLE GAP EVENT ADV COMPLETE");
        ble_app_advertise();
        break;
    default:
        break;
    }
    return 0;
}

// Fonction pour initialiser la publicité BLE
void ble_app_advertise(void)
{
    struct ble_hs_adv_fields fields;
    const char *device_name;
    memset(&fields, 0, sizeof(fields));
    device_name = ble_svc_gap_device_name();
    fields.name = (uint8_t *)device_name;
    fields.name_len = strlen(device_name);
    fields.name_is_complete = 1;
    ble_gap_adv_set_fields(&fields);

    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    ble_gap_adv_start(ble_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event, NULL);
}

// Fonction de synchronisation BLE
void ble_app_on_sync(void)
{
    ble_hs_id_infer_auto(0, &ble_addr_type);
    ble_app_advertise();
}

// Tâche principale BLE
void host_task(void *param)
{
    nimble_port_run();
}

// Fonction pour configurer le servo moteur
void setup_servo(void) {
    // Configuration du timer LEDC
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = LEDC_FREQUENCY,
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER
    };
    ledc_timer_config(&ledc_timer);

    // Configuration du canal LEDC
    ledc_channel_config_t ledc_channel = {
        .channel = LEDC_CHANNEL,
        .duty = 0,
        .gpio_num = SERVO_GPIO_PIN,
        .speed_mode = LEDC_MODE,
        .hpoint = 0,
        .timer_sel = LEDC_TIMER
    };
    ledc_channel_config(&ledc_channel);
}

void pulse_task(void *param) {
    while (1) {
        int heart_rate = read_heart_rate_sensor();

        // Afficher la valeur du BPM sur le moniteur série
        ESP_LOGI(TAG, "Fréquence cardiaque : %d BPM", heart_rate);

        // Mettre à jour l'affichage de la fréquence cardiaque sur l'écran LCD
        update_heart_rate_display(heart_rate);

        // Pause de 500 ms
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
// Fonction principale
void app_main(void)
{
     // Initialiser l'I2C pour l'écran LCD
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialisé avec succès");

    // Initialiser l'écran LCD
    lcd_init();
    lcd_set_cursor(0, 0);
    lcd_send_string("Heart rate:");

    

    // Configuration de l'ADC pour le capteur de pouls
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(PULSE_PIN, ADC_ATTEN_DB_11);

    
// Démarrer la tâche de mesure du pouls
    xTaskCreate(pulse_task, "pulse_task", 2048, NULL, 5, NULL);


    ESP_ERROR_CHECK(nvs_flash_init());
    nimble_port_init();
    ble_svc_gap_device_name_set("BLE_Device");
    
    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_gatts_count_cfg(gatt_svcs);
    ble_gatts_add_svcs(gatt_svcs);
    ble_hs_cfg.sync_cb = ble_app_on_sync;
    nimble_port_freertos_init(host_task);

    // Configurer le servo moteur
    setup_servo();

    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}