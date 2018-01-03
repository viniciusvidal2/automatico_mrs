# automatico_mrs
Aqui esta o codigo para comunicar automaticamente o controle dos servos de acordo com a missao planejada para a placa PIXHAWK, utilizando o pacote mavros, a partir dos sensores embarcados e logica calculada.

No arquivo CMakeLists.txt adicionar (porem ja adicionado na versao mais recente):

add_compile_options(-std=c++11)


Pacotes necessarios no mesmo workspace para rodar com as dependencias incluidas ate o momento:

dynamixel_workbench_msgs disponivel em https://github.com/viniciusvidal2/dynamixel-workbench-msgs
dunamixel_workbench_toolbox disponivel em https://github.com/viniciusvidal2/dynamixel-workbench

Caso envie o codigo novamente para dentro da placa, ha necessidade de reiniciar o driver da mesma, feito com os seguintes comandos:

sudo modprobe -r cdc_acm
sudo modprobe cdc_acm
