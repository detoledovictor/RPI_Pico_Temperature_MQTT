/*
* Eletrônica Embarcada com IoT, IA e Robótica - exercicio 01 - Leitor de temperatura interna do MCU RP2040 com envio via MQTT
* Aluno: Victor Hugo de Toledo Nunes
* Prof.: Gustavo Ferreira Palma 
*/

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "pico/cyw43_arch.h"

#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"
#include "bsp/board.h"
#include <string.h>

const char* WIFI_SSID = "iVictor";
const char* WIFI_PASS = "mostarda";
const char* MQTT_BROKER_IP = "35.172.255.228";
const char* MQTT_TOPIC = "temperatura/rp2040_victor";

const uint LED_PIN = 17; // LED onboard
const uint32_t INTERVALO_ENVIO_MS = 30000;

static mqtt_client_t* mqtt_cliente = NULL;

struct mqtt_connect_client_info_t info_cliente = {
    "gustavo_prof", NULL, NULL, 60, NULL, NULL, 0, 0
};

int pico_led_init(void) {
#if defined(PICO_DEFAULT_LED_PIN)
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    return PICO_OK;
#elif defined(CYW43_WL_GPIO_LED_PIN)
    return cyw43_arch_init();
#endif
}

void pico_set_led(bool led_on) {
#if defined(PICO_DEFAULT_LED_PIN)
    gpio_put(LED_PIN, led_on);
#elif defined(CYW43_WL_GPIO_LED_PIN)
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_on);
#endif
}

float read_onboard_temperature() {
    const float conversionFactor = 3.3f / (1 << 12); // 12-bit ADC
    float adc = (float)adc_read() * conversionFactor;
    float tempC = 27.0f - (adc - 0.706f) / 0.001721f;
    return tempC;
}

static void mqtt_dados_recebidos_cb(void *arg, const u8_t *dados, u16_t comprimento, u8_t flags){
    printf("Mensagem recebida (%d bytes): ", comprimento);
    for (int i = 0; i < comprimento; i++) {
        putchar(dados[i]);
    }
    printf("\n");
}

static void mqtt_chegando_publicacao_cb(void *arg, const char *topico, u32_t tamanho){
    printf("Publicação recebida no tópico: %.*s\n", tamanho, topico);
}

static void mqtt_req_cb(void *arg, err_t erro) {
    if (erro != ERR_OK) {
        printf("Erro na requisição MQTT: %d\n", erro);
    } else {
        printf("Requisição MQTT concluída com sucesso.\n");
    }
}

static void mqtt_conectado_cb(mqtt_client_t *cliente, void *arg, mqtt_connection_status_t status) {
    if (status == MQTT_CONNECT_ACCEPTED) {
        printf("Conectado ao broker MQTT!\n");
        mqtt_subscribe(cliente, MQTT_TOPIC, 0, mqtt_req_cb, NULL);
    } else {
        printf("Conexão rejeitada pelo broker, código: %d\n", status);
    }
}

int main() {
    stdio_init_all();
    int rc = pico_led_init();
    hard_assert(rc == PICO_OK);

    // Inicializa Wi-Fi
    if (cyw43_arch_init()) {
        printf("Wi-Fi init failed\n");
        return -1;
    }

    cyw43_arch_enable_sta_mode();

    printf("Conectando ao Wi-Fi...\n");
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASS, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("Falha ao conectar no Wi-Fi.\n");
        return -1;
    }
    printf("Conectado ao Wi-Fi.\n");

    // Inicializa temperatura
    adc_init();
    adc_set_temp_sensor_enabled(true);
    adc_select_input(4);

    // Cria cliente MQTT e configura callbacks
    mqtt_cliente = mqtt_client_new();
    mqtt_set_inpub_callback(mqtt_cliente, mqtt_chegando_publicacao_cb, mqtt_dados_recebidos_cb, NULL);

    ip_addr_t endereco_broker;
    ip4addr_aton(MQTT_BROKER_IP, &endereco_broker);

    err_t erro = mqtt_client_connect(mqtt_cliente, &endereco_broker, 1883, mqtt_conectado_cb, NULL, &info_cliente);
    if (erro != ERR_OK) {
        printf("Erro ao iniciar conexão com o broker MQTT.\n");
        return -1;
    }

    // Loop principal
    absolute_time_t proximo_envio = make_timeout_time_ms(INTERVALO_ENVIO_MS);

    while (true) {
        cyw43_arch_poll();

        if (mqtt_client_is_connected(mqtt_cliente) && absolute_time_diff_us(get_absolute_time(), proximo_envio) < 0) {
            float temp = read_onboard_temperature();
            char payload[64];
            snprintf(payload, sizeof(payload), "Temperatura: %.2f C", temp);

            err_t erro_pub = mqtt_publish(mqtt_cliente, MQTT_TOPIC, payload, strlen(payload), 0, 0, mqtt_req_cb, NULL);
            if (erro_pub == ERR_OK) {
                printf("Temperatura enviada: %s\n", payload);
                pico_set_led(true);
                sleep_ms(100);
                pico_set_led(false);
            } else {
                printf("Erro ao enviar temperatura\n");
            }

            proximo_envio = make_timeout_time_ms(INTERVALO_ENVIO_MS);
        }

        sleep_ms(100);
    }
}
