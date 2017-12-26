# automatico_mrs
Aqui esta o codigo para comunicar automaticamente o controle dos servos de acordo com a missao planejada para a placa PIXHAWK, utilizando o pacote mavros, a partir dos sensores embarcados e logica calculada.

FACA TUDO EM WORKSPACE SEPARADO, porque da conflito de compilador, precisa de suporte para c++11

Necessario adicionar no qt, na aba Build, para CatkinMake Arguments:

-DCMAKE_C_COMPILER=/usr/bin/gcc -DCMAKE_CXX_COMPILER=/usr/bin/gcc -v

Pacotes necessarios no mesmo workspace para rodar com as dependencias incluidas ate o momento:

dynamixel_workbench_msgs disponivel em https://github.com/viniciusvidal2/dynamixel-workbench-msgs
dunamixel_workbench_toolbox disponivel em https://github.com/viniciusvidal2/dynamixel-workbench, mas somente a pasta toolbox, nao o diretorio todo, pois havera conflito
