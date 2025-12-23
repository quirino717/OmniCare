# OmniCare Bringup

O **omnicare_bringup** é o pacote responsável pela **inicialização completa do OmniCare**, tanto no **robô real** quanto no **ambiente de simulação**.  
Ele centraliza todos os *launch files* necessários para subir o sistema de forma consistente, garantindo que sensores, controladores, interfaces de hardware e módulos de software estejam corretamente configurados antes da execução das missões.

Este pacote é o ponto de entrada principal para a operação do robô.

---

## Considerações

O bringup foi projetado para:

- Facilitar a inicialização do sistema com um único comando;
- Garantir consistência entre simulação e robô real;
- Reduzir erros de configuração manual;
- Centralizar parâmetros críticos de inicialização.

Sempre que possível, a lógica de inicialização é mantida no `omnicare_bringup`, evitando que outros pacotes precisem lidar diretamente com dependências de hardware ou configuração global do sistema.

---

## Arquitetura Geral

O pacote atua como um **orquestrador de inicialização**, sendo responsável por subir e interligar os seguintes componentes:

- Descrição do robô (URDF);
- ROS 2 Control e controladores;
- Comunicação com microcontroladores (STM32);
- Sensores embarcados (ex.: LiDAR);
- Interfaces necessárias para navegação e controle.

Ele não implementa lógica de comportamento ou navegação, apenas prepara o ambiente para que esses módulos possam operar corretamente.

---

## Funcionalidades Principais

O `omnicare_bringup` é responsável por:

- Inicializar o **ROS 2 Control**, carregando controladores e interfaces de hardware;
- Estabelecer a comunicação com os **microcontroladores da base**;
- Inicializar sensores essenciais, como o **LiDAR**;
- Carregar a descrição física do robô;

---

## Launch Files

Os principais *launch files* disponíveis neste pacote incluem:

- **`load_real_robot.launch.py`**  
  Responsável por inicializar o OmniCare no robô real, incluindo:
  - ROS 2 Control;
  - Interfaces de hardware;
  - Sensores físicos;
  - Configurações específicas do ambiente real.

- **`load_sim_robot.launch.py`**  
  Responsável por inicializar o OmniCare em **simulação**, incluindo:
    - Carregar o modelo do robô no Gazebo;
    - Publicar corretamente o URDF via `robot_state_publisher`;
    - Inicializar sensores e estimadores em modo simulado;
    - Permitir a escolha dinâmica do ambiente (FEI ou HU);
    - Disponibilizar ferramentas de depuração, como o RViz.

<br>


---

## Como Executar

### Robô Real
```bash
ros2 launch omnicare_bringup load_real_robot.launch.py
```

### Simulação
```bash
ros2 launch omnicare_bringup load_sim_robot.launch.py
```

Após a execução de qualquer um dos modos, o robô estará pronto para receber comandos de teleoperação ou iniciar missões automáticas através do Behavior Manager.

---

## Papel no Sistema

O **omnicare_bringup** é responsável por **preparar todo o ecossistema do OmniCare para operação**.  
Sem ele, os pacotes funcionariam apenas de forma isolada; com ele, o sistema é iniciado de maneira estruturada, confiável e reprodutível, tanto em simulação quanto no robô real.