# 🚨 Guia de LEDs e Monitoramento de Bateria

## 📋 Tabela de Status dos LEDs

A placa **Tapejara TPJ-01** tem 4 LEDs, um por braço: **2 verdes** (M2, M3) e **2 vermelhos**
(M1, M4). Não há LED azul como na placa anterior, então `LEDControl` agrupa os LEDs em dois
canais lógicos e resolve o canal de status por prioridade.

| Status | LEDs | Ação | Descrição |
|--------|------|------|-----------|
| **SENSORS_CALIBRATION** | 🟢 VERDES | Blinking slowly (1 s) | Calibrando sensores (inicialização) |
| **SYSTEM_READY** | 🟢 VERDES | Blinking (500 ms) | Sistema pronto, aguardando conexão |
| **UDP_RX** | 🟢 VERDES | Fully lit | App conectado, recebendo pacotes UDP |
| **LOW_POWER** | 🔴 VERMELHOS | Fully lit | Bateria baixa ou crítica |

Prioridade do canal de status: `UDP_RX` > `SYSTEM_READY` > `SENSORS_CALIBRATION`.

## 🔌 Mapeamento de Pinos

Fonte de verdade: [`include/board_config.h`](../include/board_config.h). Esquemático:
`tapejaraBoard_v1.pdf`, bloco *Sinalization*.

```cpp
IO6   -> LED1 (vermelho, braço M1)  ─┐ canal ALERTA
IO5   -> LED4 (vermelho, braço M4)  ─┘
IO41  -> LED2 (verde, braço M2)     ─┐ canal STATUS
IO43  -> LED3 (verde, braço M3)     ─┘

IO1   -> ADC de bateria (divisor R22/R23 = 100k/100k, VBAT/2)
```

> **IO43 era o U0TXD.** Por isso o build define `-DARDUINO_USB_CDC_ON_BOOT=1`: o `Serial`
> precisa sair pelo USB nativo (IO19/20), e não pelo UART0. Durante o reset o ROM bootloader
> ainda escreve seu log nesse pino, então o LED3 pisca por um instante no boot — é normal.

## 🔋 Sistema de Monitoramento de Bateria

### Thresholds de Tensão (LiPo 1S - 3.7V Nominal)

- **Tensão Plena**: 4.2V (100% - totalmente carregada)
- **Tensão Nominal**: 3.7V (~50% - tensão de armazenamento)
- **Tensão Normal**: > 3.4V (uso seguro)
- **⚠️ Bateria Baixa**: 3.4V (~20% restante - LED vermelho acende)
- **🚨 Bateria Crítica**: 3.2V (mínimo seguro - motores desarmam automaticamente)
- **⛔ NUNCA descarregue abaixo de 3.0V** (danifica permanentemente a bateria)

### Leitura de Tensão

O pino IO1 (ADC1_CH0) lê a tensão da bateria através do divisor R22/R23 = 100k/100k (VBAT/2).

**Cálculo:**
```
ADC lê 0-3.3V (0-4095)
Tensão no ADC = (adc_value / 4095) × 3.3V
Tensão da bateria = Tensão_ADC × 2.0 (divisor)
```

### Proteções Implementadas

1. **Inicialização Bloqueada**: Se bateria < 3.0V, sistema não inicia
2. **Desarme Automático**: Se bateria cai para < 3.0V durante voo, motores param
3. **Alerta Visual**: LED vermelho acende quando < 3.3V
4. **Monitoramento Contínuo**: Verifica bateria a cada 1 segundo

## 🎯 Comportamento dos LEDs

### Durante Inicialização

1. **LED Branco acende automaticamente** → Alimentação presente (hardware)
2. **Verifica bateria** → Se crítica, LED vermelho acende e trava
3. **LED Azul pisca lentamente** → Calibrando sensores
4. **LED Azul apaga** → Calibração concluída
5. **Aguarda conexão WiFi** → LEDs estáveis

### Durante Operação Normal

- **LED Branco**: Sempre aceso (não controlável - hardware)
- **LED Azul**: Pisca quando cliente WiFi conecta
- **LED Verde**: Pisca quando recebe pacotes UDP
- **LED Vermelho**: Acende se bateria < 3.4V

### Situações de Emergência

#### Bateria Baixa (3.2V - 3.4V)
- 🔴 LED vermelho ACESO
- ⚠️ Sistema continua operando
- 📢 Aviso no Serial Monitor
- 🔋 ~20% de carga restante - **POUSE EM BREVE**

#### Bateria Crítica (< 3.2V)
- 🔴 LED vermelho ACESO
- 🛑 Motores DESARMAM automaticamente
- 🚨 Sistema entra em modo seguro
- 📢 Alerta crítico no Serial Monitor
- ⚠️ **RISCO DE DANO À BATERIA**

## 📊 Informações no Serial Monitor

O sistema exibe a cada segundo:

```
╔════════════════════════════════════════════════════════╗
║                   STATUS DOS LEDS                      ║
╚════════════════════════════════════════════════════════╝
⚪ WHITE (Power):       FULLY_LIT
🔵 BLUE (System):      BLINKING
🟢 GREEN (UDP):        BLINKING
🔴 RED (Low Battery):  OFF

╔════════════════════════════════════════════════════════╗
║                  STATUS DA BATERIA                     ║
╚════════════════════════════════════════════════════════╝
🔋 Tensão:      3.85V
📊 Carga:       70.8%
✅ Status:      OK
```

## ⚙️ Configuração

### Ajustar Thresholds de Bateria

Em `lib/utils/led_control.h`:

```cpp
#define BATTERY_LOW_VOLTAGE   3.4f    // Tensão considerada baixa (~20%)
#define BATTERY_CRITICAL_VOLTAGE 3.2f // Tensão crítica (mínimo seguro)
#define BATTERY_FULL_VOLTAGE  4.2f    // Tensão máxima (totalmente carregada)
#define BATTERY_NOMINAL_VOLTAGE 3.7f  // Tensão nominal (armazenamento)
```

### Ajustar Velocidade de Blinking

Em `lib/utils/led_control.cpp`:

```cpp
blink_interval = 500;  // Blinking normal (ms)
blink_interval = 1000; // Blinking lento (ms)
```

## 🧪 Teste do Sistema

Para testar os LEDs e bateria:

```cpp
void testLEDs() {
    leds.setPowerOn(true);
    delay(1000);
    
    leds.setSensorsCalibration(true);
    delay(3000);
    
    leds.setSystemReady(true);
    delay(3000);
    
    leds.setUDPReceiving(true);
    delay(3000);
    
    leds.setLowPower(true);
    delay(3000);
    
    // Verifica bateria
    Serial.printf("Bateria: %.2fV (%.1f%%)\n", 
                  leds.getBatteryVoltage(), 
                  leds.getBatteryPercentage());
}
```

## 🔧 Troubleshooting

### LED não acende
- Verifique conexão física
- Confirme pino correto no código
- Teste com multímetro

### Leitura de bateria incorreta
- Verifique divisor de tensão (deve ser VBAT/2)
- Calibre ADC se necessário
- Confirme que IO1 está conectado corretamente

### LEDs piscam erraticamente
- Verifique fonte de alimentação
- Adicione capacitor de filtro
- Reduza consumo de corrente

## 📝 Notas Importantes

1. **Nunca** opere com bateria < 3.2V (danifica LiPo e reduz vida útil)
2. **Sempre** verifique tensão antes de voar (mínimo 3.5V recomendado)
3. **Pouse imediatamente** quando LED vermelho acender (< 3.4V)
4. **Recarregue** quando atingir 3.4V (~20% restante)
5. **Armazene** LiPo em 3.7-3.8V (~50% de carga) para longevidade
6. **Não deixe** descarregada por longos períodos

### 🔋 Tabela de Referência (LiPo 1S - 3.7V)

| Tensão | Carga | Status | Ação |
|--------|-------|--------|------|
| 4.20V | 100% | ✅ Totalmente carregada | Pronto para voar |
| 4.00V | ~87% | ✅ Ótimo | Voo normal |
| 3.85V | ~71% | ✅ Bom | Voo normal |
| 3.70V | ~50% | ✅ OK | Nominal - armazenamento ideal |
| 3.60V | ~33% | ⚠️ Atenção | Considere pousar em breve |
| 3.40V | ~20% | 🔴 Baixa | **LED vermelho** - Pouse agora! |
| 3.20V | ~5% | 🚨 Crítica | **Motores desarmam** - Perigo! |
| <3.00V | 0% | ⛔ Danificada | Bateria pode estar permanentemente danificada |

## 🎓 Referências

- **Especificações LiPo 1S (3.7V nominal):**
  - Tensão máxima (carregada): 4.2V
  - Tensão nominal: 3.7V
  - Tensão mínima segura: 3.2V
  - Tensão de descarga absoluta: 3.0V (evitar!)
  
- **ADC ESP32:** 12-bit (0-4095), faixa 0-3.3V
- **Divisor de tensão:** R1=R2 → VBAT/2
- **Curva de descarga LiPo:** Não-linear (cai rapidamente abaixo de 3.5V)
