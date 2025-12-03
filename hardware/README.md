# OmniCare Hardware

O hardware da OmniCare foi projetado para ser **modular, confiável e duradouro**.  
Neste diretório estão reunidos todos os arquivos relacionados ao desenvolvimento elétrico do projeto.  
A seguir, é apresentada a arquitetura geral que descreve como a parte elétrica foi concebida.

## Considerações

Todo o projeto elétrico foi desenvolvido utilizando o software **Altium Designer**, versão *24.8.2*  
(até o momento, ainda é possível utilizar o e-mail institucional da FEI para acesso).

Com exceção da *Driver Board*, todas as placas foram manufaturadas no **CLE da FEI**.  
As placas de duas faces foram projetadas para que os componentes sejam soldados apenas na face **Bottom**,  
utilizando *vias* para permitir a passagem de sinais entre as camadas. Essas vias foram planejadas de forma a permitir o uso de qualquer condutor simples (por exemplo: terminais de resistores ou fios finos).

Dentro de cada pasta estão incluídos:  
– **Arquivos de fabricação**, identificando o local de produção (CLE ou JLCPCB);  
– **PDF de documentação**;  
– **Arquivos do Altium** correspondentes ao projeto.

## Arquitetura Geral


## Placas Eletrônicas da OmniCare

### Power Board
Placa responsável por **receber a energia da bateria**, realizar a **regulação das tensões** utilizadas no robô e fazer a **distribuição segura** para todos os módulos.

### Driver Board
Placa destinada ao **acionamento dos motores**, contendo os circuitos de potência e controle necessários para operar as rodas omnidirecionais com precisão e confiabilidade.

### Base Board
Placa responsável por **interligar os módulos eletrônicos da base**, roteando sinais dos motores, encoders e sensores, além de fornecer **pontos de conexão e organização elétrica** entre os subsistemas.

### IHR Board (HRI Board)
Placa responsável pela **interação humano–robô**, integrando LEDs, botões e elementos visuais utilizados para comunicação e feedback ao usuário.

### Manip Board
Placa dedicada ao **controle do manipulador frontal**, gerenciando motores e sensores usados para acionar os botões do elevador de forma precisa.
