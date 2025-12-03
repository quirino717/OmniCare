# OmniCare Hardware

O hardware da OmniCare foi projetado para ser **modular, confiável e duradouro**.  
Neste diretório estão reunidos todos os arquivos relacionados ao desenvolvimento elétrico do projeto.  
Abaixo é apresentada a arquitetura geral que descreve como a parte elétrica foi concebida.

## Considerações

Todo o projeto elétrico foi desenvolvido utilizando o software **Altium Designer**, versão *24.8.2*  
(até o presente momento é possível utilizar o e-mail institucional da FEI para acesso).

Com exceção da *Driver Board*, todas as placas foram manufaturadas no **CLE da FEI**.  
As placas que possuem duas faces foram projetadas para que os componentes sejam soldados apenas na face **Bottom**,  
utilizando *vias* para permitir a passagem de sinais entre as camadas. Essas vias foram planejadas de forma a permitir o uso de qualquer condutor simples (por exemplo: terminais de resistores ou fios finos).

Dentro de cada pasta contem: os arquivos de fabricaçao e identificado com qual local foi fabricado (CLE ou JLCPCB), o PDF de documentaçao e os arquivos do Altium.


## Arquitetura Geral


## Placas Eletrônicas da OmniCare
### Power Board
Placa responsável por **receber a energia da bateria**, realizar a **regulação das tensões** usadas no robô e fazer a **distribuição segura** para todos os módulos.

### Driver Board
Placa destinada ao **acionamento dos motores**, contendo os circuitos de potência e controle necessários para operar as rodas omnidirecionais de forma precisa e confiável.

### Base Board
Placa responsável por **interligar os módulos eletrônicos da base**, roteando sinais dos motores, encoders e sensores, além de fornecer **conexões e organização elétrica** entre os subsistemas.


### IHR Board (HRI Board)
Placa responsável pela **interação humano–robô**, integrando LEDs, botões e elementos visuais utilizados para comunicação e feedback ao usuário.

### Manip Board
Placa dedicada ao **controle do manipulador frontal**, gerenciando motores e sensores usados para acionar os botões do elevador de forma precisa.


