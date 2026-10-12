# ⌚ SAFE-FALL — Wearable Fall Detection Wristband

O **SAFE-FALL** é um dispositivo vestível (*wearable*) de deteção inteligente de quedas e monitorização de emergência, desenvolvido sobre o microcontrolador **ESP32-C6**. O sistema integra sensorização inercial, reconhecimento de voz e alertas em tempo real na nuvem.

---

## 🚀 Funcionalidades Principais

- **Deteção de Quedas via IMU:** Leitura e processamento de aceleração e rotação angular através do sensor IMU **MPU6050**.
- **Reconhecimento de Voz / Áudio:** Captação de áudio via microfone **I2S** com modelo de Inteligência Artificial / Machine Learning incorporado (desenvolvido via Edge Impulse) para identificação de pedidos de socorro.
- **Alertas na Nuvem:** Envio instantâneo de notificações de emergência via plataforma **Blynk IoT**.
- **Conectividade Eficiente:** Microcontrolador ESP32-C6 (SuperMini) com suporte para conectividade Wi-Fi/BLE e gestão eficiente de energia.

---

## 📁 Apresentação e Documentação do Projeto

-  **[Consultar / Descarregar Apresentação do Projeto (Google Drive)](https://drive.google.com/drive/folders/1ZoIe0bks5Wsaky01tTAF8UVRqMZX4R7U?usp=drive_link)**
- **Modelo de Áudio Impulse:** Ficheiro de biblioteca Arduino exportado (`ei-safefall-voice-recognition-arduino-1.0.1-impulse-#1.zip`).

---

## 🛠️ Hardware & Componentes

| Componente | Função |
| :--- | :--- |
| **ESP32-C6 / SuperMini** | Unidade Central de Processamento e Conectividade |
| **MPU6050** | Acelerómetro e Giroscópio 6-Eixos |
| **Microfone I2S** | Captação de voz e deteção de comandos de socorro |
| **Blynk IoT** | Plataforma de Gestão de Alertas na Nuvem |

---

## 💻 Estrutura do Código

- `projeto_final_ESP32_C6_MINI1/`: Firmware principal desenvolvido em C++/Arduino para o microcontrolador ESP32-C6.
