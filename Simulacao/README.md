# Simulação

Diretório que contém códigos para simulação
de um motor BLDC em linguagem python e C.

A simulação em linguagem  C está mais completar em requisitos de 
parâmetros.

# Simulações

Para realizar as simulações basta rodar o seguinte comando:

```bash
gcc c/main.c -o c/NOME.out -lm && ./c/NOME.out -c MOTOR_PARAM.txt  && python3 plot.py closedloop_simulation.csv DIRETORIO_IMAGENS
```
Os parâmetros do motor e outros parâmetros de simulações pode ser configurados em um arquivo _.txt_ e passado via linha de 
comando ao executar o programa. O arquivo precisa estar nesse estilo:

```txt
# ==========================================================
#   Arquivo de parametros da simulacao closedloop
#   Uso: ./xx.out -c ESTE_ARQUIVO.txt
#
#   Formato: CHAVE=VALOR (tambem aceita "CHAVE VALOR")
#   Linhas em branco ou iniciadas com '#' sao ignoradas.
#   Prioridade: argumentos de linha de comando > este arquivo > default
# ==========================================================

# --- Parametros do motor ---
R  = 0.053           # Ohm  - resistencia de armadura
L  = 0.045e-3        # H    - indutancia de magnetizacao
M  = 0.0            # H    - indutancia mutua
Ke = 0.0682         # V/(rad/s) - constante eletrica
J  = 1.8e-4         # kg.m^2 - momento de inercia
B  = 1.2e-6         # coeficiente de amortecimento
Tl = 0.0            # N.m  - torque de carga
P  = 14             # numero de pares de polos
Kt = 0.0682         # N.m/A - constante de torque

# --- Parametros do SVPWM (chaveamento real do inversor) ---
Fsw        = 10000  # Hz - frequencia de chaveamento
PwmSamples = 200    # passos finos de simulacao por periodo Ts
                    # (maior = mais fiel ao chaveamento real, porem mais lento)

# --- Parametros de tempo da simulacao ---
Ti = 0.0                 # s - tempo inicial
Tf = 0.25                # s - tempo final
Dt = 1e-6                # s - passo de integracao explicito
                         # (0.0 = automatico: dt = 1/Fsw / PwmSamples;
                         #  informe um valor > 0 para sobrescrever o
                         #  calculo automatico)

# --- Barramento CC ---
Vdc = 36.0               # V - tensao do barramento CC

# --- Controladores PI ---
KpOmega = 2.0            # ganho proporcional - malha de velocidade
KiOmega = 0.5            # ganho integral      - malha de velocidade
KpId    = 3.0            # ganho proporcional - malha de corrente id
KiId    = 0.8            # ganho integral      - malha de corrente id
KpIq    = 3.0            # ganho proporcional - malha de corrente iq
KiIq    = 0.8            # ganho integral      - malha de corrente iq

# --- Referencia de velocidade ---
rpm=200

# --- Partida V/F em malha aberta ---
# 1 (ou true/yes/on)  -> parte com rampa V/F em malha aberta (theta_e
#                        sintetico, iniciando em 0 rad) e depois comuta
#                        para FOC em malha fechada. E o comportamento
#                        default caso esta chave nem seja informada.
# 0 (ou false/no/off) -> pula a partida V/F: roda o FOC em malha
#                        fechada (theta_e = motor.theta_r * P) desde
#                        Ti, igual ao fluxo anterior a implementacao
#                        do V/F.
VfStartup = 0

# -- Partida em malha aberta --
# Sera gerado ondas de alimentação ideias senoidais
# que alimentara o motor.
# A logica de acionamento do parametro é a mesma do
# parametro "VfStartup".
# Somente uma das flags deve estar ativada: ou A MalhaAberta
# ou a VfStartup, as duas com valor True/1/on nao pode.
#
MalhaAberta = 0

# --- Saida ---
file = closedloop_simulation.csv
```

Caso esteja utilizando ambiente **LINUX**, pode-se utilizar do [makefile](./makefile) presente no diretório, basta mudar as variáveis para o que deseja e executar a ação desejada.

# Log de Saída da Simulação (CSV)

O arquivo CSV gerado pela simulação (`closedloop_simulation.csv` por padrão)
usa `;` como separador de campos. Cada linha representa um passo de
integração (`dt`) da simulação.

## Variáveis

| Campo | Unidade | Descrição |
|---|---|---|
| `time` | s | Instante de tempo da simulação |
| `mode` | — | Fase de operação: `0` = partida V/F em malha aberta, `1` = FOC em malha fechada |
| `Va`, `Vb`, `Vc` | V | Tensões de fase aplicadas ao motor (saída real do inversor, já chaveada) |
| `ia`, `ib`, `ic` | A | Correntes de fase do motor (abc) |
| `id`, `iq` | A | Correntes no referencial síncrono dq (Park). `id` idealmente ≈ 0; `iq` é proporcional ao torque |
| `Te` | N·m | Torque eletromagnético desenvolvido pelo motor |
| `theta_r` | rad | Posição angular mecânica do rotor (acumulada, não normalizada) |
| `omega_r` | rad/s | Velocidade angular mecânica do rotor |
| `iq_ref` | A | Referência de corrente `iq` gerada pela malha de velocidade (saída do PI de velocidade) |
| `vd_ref`, `vq_ref` | V | Tensões de referência em dq geradas pelas malhas de corrente (antes do SVPWM) |
| `duty_a`, `duty_b`, `duty_c` | 0–1 | Duty cycles de referência de cada braço do inversor (saída do SVPWM) |
| `carrier` | 0–1 | Valor instantâneo da portadora triangular do PWM |
| `gate_a`, `gate_b`, `gate_c` | 0 ou 1 | Estado de chaveamento da chave superior de cada braço (1 = ligada) |
| `sat_omega` | 0 ou 1 | *(somente build DEBUG)* Indica se o PI de velocidade está saturado no passo |
| `sat_d` | 0 ou 1 | *(somente build DEBUG)* Indica se o PI de corrente `id` está saturado no passo |
| `sat_q` | 0 ou 1 | *(somente build DEBUG)* Indica se o PI de corrente `iq` está saturado no passo |

## Observações

- `Va/Vb/Vc` e `gate_a/b/c` refletem o **chaveamento real** do inversor
  (não a tensão média), portanto apresentam ripple de alta frequência
  mesmo em regime permanente — isso é esperado.
- `vd_ref`/`vq_ref` são úteis para avaliar a tensão "útil" pedida pelo
  controle, sem o ripple do PWM.
- Os campos `sat_omega`, `sat_d`, `sat_q` só aparecem quando o binário é
  compilado com `-DDEBUG` (ou `DEBUG=1`); no build padrão essas três
  colunas não são geradas.
- Durante a fase de partida V/F (`mode = 0`), os campos `id`, `iq`,
  `iq_ref`, `vd_ref`, `vq_ref` não refletem a malha FOC (que ainda não
  está ativa) — `theta_e` usado nesse trecho é sintético, não vem da
  posição real do rotor.


# Referências utilizadas

## Ajuste do Controlador PI

1. M. Rad, F. Şimonca, A. Frătean and P. Dobra, "Embedded speed control of BLDC motors using LPC1549 microcontroller," 2016 IEEE International Conference on Automation, Quality and Testing, Robotics (AQTR), Cluj-Napoca, Romania, 2016, pp. 1-5, doi: 10.1109/AQTR.2016.7501341. keywords: {Brushless DC motors;Sensors;Microcontrollers;Motor drives;Transfer functions;BLDC motor;root locus;software configurable timer;Hall sensor;speed control}

2. F. R. Rahman, A. S. Rohman, I. Munawar and S. Sereyvatha, "Speed Control System of BLDC Motor using PI Anti – Windup Controller on an Autonomous Vehicle Prototype (AVP)," 2018 IEEE 8th International Conference on System Engineering and Technology (ICSET), Bandung, Indonesia, 2018, pp. 51-56, doi: 10.1109/ICSEngT.2018.8606398. keywords: {DC motors;Pulse width modulation;Permanent magnet motors;Pins;Velocity control;Windup;PI Anti –Windup;saturation;PWM;setpoint;MATLAB;BLDC Motor}

## Modelagem do motor BLDC

1. S. A. Zabalawi and A. Nasiri, "State Space Modeling and Simulation of Sensorless Control of Brushless DC Motors Using Instantaneous Rotor Position Tracking," 2007 IEEE Vehicle Power and Propulsion Conference, Arlington, TX, USA, 2007, pp. 90-94, doi: 10.1109/VPPC.2007.4544104.
keywords: {State-space methods;Sensorless control;Brushless DC motors;Rotors;DC motors;Reluctance motors;Hysteresis motors;Traction motors;Power system reliability;Mathematical model;Brushless machine;Sensorless;Simulation;State Space}

