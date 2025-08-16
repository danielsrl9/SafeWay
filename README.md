# SafeWay 🪖
Capacete inteligente que detecta impactos e envia alertas via Bluetooth.

---

## 📖 Descrição
O SafeWay é um projeto open-source que utiliza **ESP32** e **MPU6050** para monitorar aceleração e rotação, detectando quedas ou colisões.  
Quando um evento crítico é detectado, o sistema envia um alerta em tempo real para um smartphone conectado via Bluetooth.  

Este é um protótipo inicial (MVP), ideal para testes e desenvolvimento, sem integração com GPS ou envio de mensagens automáticas para contatos de emergência.

---

## ⚙️ Funcionalidades principais
- 📡 Detecção de impactos graves (queda ou colisão)  
- 📲 Envio de alerta via Bluetooth para smartphone conectado  
- 🪪 Dispositivo Bluetooth identificado como `"SafeWayHelmet"`  
- 🔔 Mensagens de depuração no monitor serial  
- 🛠 Thresholds ajustáveis para aceleração e rotação (LOW_G_THRESHOLD, HIGH_G_THRESHOLD, ROTATION_THRESHOLD)  

---

