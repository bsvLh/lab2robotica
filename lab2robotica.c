/*
 * Controlador en C para Webots
 * Laboratorio 2: Navegación reactiva con filtrado y fusión de sensores
 */

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/position_sensor.h>
#include <stdio.h>
#include <math.h>

// --- Constantes del Robot y Simulación ---
#define TIME_STEP 64
#define WHEEL_RADIUS 0.0205 // Metros
#define MAX_SPEED 6.28      // Rad/s
#define CRUISE_SPEED 3.0    // Rad/s
#define TURN_SPEED 2.0      // Rad/s

#define MAX_SENSOR_DIST 0.07 // Rango máximo confiable del e-puck IR (7 cm)
#define SAFE_DISTANCE 0.035  // Umbral de decisión para evadir obstáculos

// --- Parámetros del Filtro de Kalman ---
#define KALMAN_Q 0.0001 // Varianza del proceso (confianza en encoders)
#define KALMAN_R 0.005  // Varianza de la medición (ruido del sensor)

// Estructura para el Filtro de Kalman 1D
typedef struct {
    double x; // Estimación de la distancia
    double p; // Varianza (incertidumbre)
} KalmanFilter;

// Estructura para la tabla de conversión IR a metros
typedef struct {
    double distance;
    double raw;
} SensorLookup;

SensorLookup lookup_table[] = {
    {0.000, 4095.0}, {0.005, 2133.3}, {0.010, 1465.7}, {0.015, 601.5},
    {0.020, 383.8},  {0.030, 234.9},  {0.040, 158.0},  {0.050, 120.0},
    {0.060, 104.1},  {0.070, 67.2}
};
const int TABLE_SIZE = 10;

// --- Funciones Auxiliares ---

// Limitar valores dentro de un rango
double clamp(double value, double min, double max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

// Convertir valor crudo del sensor IR a metros usando interpolación lineal
double raw_to_meters(double raw) {
    if (raw <= 67.2) return MAX_SENSOR_DIST; // Sin obstáculo en rango
    if (raw >= 4095.0) return 0.0;
    
    for (int i = 0; i < TABLE_SIZE - 1; i++) {
        if (raw <= lookup_table[i].raw && raw >= lookup_table[i+1].raw) {
            double alpha = (raw - lookup_table[i].raw) / (lookup_table[i+1].raw - lookup_table[i].raw);
            return lookup_table[i].distance + alpha * (lookup_table[i+1].distance - lookup_table[i].distance);
        }
    }
    return MAX_SENSOR_DIST;
}

// --- Funciones del Filtro de Kalman ---

// Etapa de Predicción: El obstáculo se acerca según lo que avanza el robot
void kf_predict(KalmanFilter *kf, double delta_s) {
    kf->x = clamp(kf->x - delta_s, 0.0, MAX_SENSOR_DIST);
    kf->p = kf->p + KALMAN_Q;
}

// Etapa de Corrección: Ajustar la predicción con la lectura real del sensor
void kf_update(KalmanFilter *kf, double measurement) {
    double K = kf->p / (kf->p + KALMAN_R); // Ganancia de Kalman
    kf->x = kf->x + K * (measurement - kf->x);
    kf->p = (1.0 - K) * kf->p;
}

// --- Función Principal ---
int main(int argc, char **argv) {
    wb_robot_init();

    // Inicialización de motores
    WbDeviceTag motor_left = wb_robot_get_device("left wheel motor");
    WbDeviceTag motor_right = wb_robot_get_device("right wheel motor");
    wb_motor_set_position(motor_left, INFINITY);
    wb_motor_set_position(motor_right, INFINITY);
    wb_motor_set_velocity(motor_left, 0.0);
    wb_motor_set_velocity(motor_right, 0.0);

    // Inicialización de encoders
    WbDeviceTag encoder_left = wb_robot_get_device("left wheel sensor");
    WbDeviceTag encoder_right = wb_robot_get_device("right wheel sensor");
    wb_position_sensor_enable(encoder_left, TIME_STEP);
    wb_position_sensor_enable(encoder_right, TIME_STEP);

    // Inicialización de sensores de distancia (ps0 y ps7 frontales; ps2 y ps5 laterales)
    WbDeviceTag ps[8];
    char ps_names[8][4];
    for (int i = 0; i < 8; i++) {
        sprintf(ps_names[i], "ps%d", i);
        ps[i] = wb_robot_get_device(ps_names[i]);
        wb_distance_sensor_enable(ps[i], TIME_STEP);
    }

    // Variables de estado
    double prev_enc_left = 0.0, prev_enc_right = 0.0;
    bool encoders_initialized = false;
    
    // Variables para el filtro simple (Media Móvil Exponencial - EMA)
    double ema_front = MAX_SENSOR_DIST;
    double alpha_ema = 0.4; // Factor de suavizado (0 a 1)

    // Inicializar Filtro de Kalman
    KalmanFilter kf = {MAX_SENSOR_DIST, 1.0};

    printf("[*] Controlador iniciado. Frecuencia de muestreo: %.1f Hz\n", 1000.0/TIME_STEP);

    // Bucle principal
    while (wb_robot_step(TIME_STEP) != -1) {
        
        // 1. LECTURA DE ENCODERS Y PREDICCIÓN (Modelo cinemático)
        double curr_enc_left = wb_position_sensor_get_value(encoder_left);
        double curr_enc_right = wb_position_sensor_get_value(encoder_right);
        double delta_s = 0.0;

        if (!encoders_initialized) {
            prev_enc_left = curr_enc_left;
            prev_enc_right = curr_enc_right;
            encoders_initialized = true;
        } else {
            double delta_l = curr_enc_left - prev_enc_left;
            double delta_r = curr_enc_right - prev_enc_right;
            delta_s = ((delta_l + delta_r) / 2.0) * WHEEL_RADIUS; // Avance lineal estimado
            
            prev_enc_left = curr_enc_left;
            prev_enc_right = curr_enc_right;
        }

        // Ejecutar predicción de Kalman con el avance
        kf_predict(&kf, delta_s);

        // 2. LECTURA DE SENSORES Y FILTRADO SIMPLE
        double raw_ps0 = wb_distance_sensor_get_value(ps[0]); // Frontal derecho
        double raw_ps7 = wb_distance_sensor_get_value(ps[7]); // Frontal izquierdo
        
        double dist_front_right = raw_to_meters(raw_ps0);
        double dist_front_left = raw_to_meters(raw_ps7);
        
        // Tomamos la distancia del obstáculo más cercano al frente
        double raw_measurement = fmin(dist_front_right, dist_front_left);
        
        // Aplicar Filtro Simple (EMA) a las lecturas
        ema_front = alpha_ema * raw_measurement + (1.0 - alpha_ema) * ema_front;

        // 3. ACTUALIZACIÓN DEL FILTRO DE KALMAN
        kf_update(&kf, raw_measurement);

        // 4. LÓGICA DE NAVEGACIÓN REACTIVA
        double speed_l = CRUISE_SPEED;
        double speed_r = CRUISE_SPEED;

        // Decisión usando la estimación de Kalman fusionada
        if (kf.x < SAFE_DISTANCE) {
            // Obstáculo detectado, verificar sensores laterales para decidir giro
            double raw_ps5 = wb_distance_sensor_get_value(ps[5]); // Izquierda
            double raw_ps2 = wb_distance_sensor_get_value(ps[2]); // Derecha
            
            // En e-puck, mayor valor crudo = obstáculo más cerca
            if (raw_ps5 > raw_ps2) {
                // Obstáculo más cerca por la izquierda -> Girar a la derecha
                speed_l = CRUISE_SPEED;
                speed_r = -TURN_SPEED;
            } else {
                // Obstáculo más cerca por la derecha -> Girar a la izquierda
                speed_l = -TURN_SPEED;
                speed_r = CRUISE_SPEED;
            }
        }

        wb_motor_set_velocity(motor_left, speed_l);
        wb_motor_set_velocity(motor_right, speed_r);

        // 5. REGISTRO Y MONITOREO (Imprimir cada ~0.5 segundos)
        int step_count = (int)(wb_robot_get_time() * 1000) / TIME_STEP;
        if (step_count % 8 == 0) {
            printf("[Data] Crudo: %.3fm | Filtro Simple(EMA): %.3fm | Kalman: %.3fm | Avance dS: %.4fm\n", 
                   raw_measurement, ema_front, kf.x, delta_s);
        }
    }

    wb_robot_cleanup();
    return 0;
}