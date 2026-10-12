#!/bin/bash

NUM_SIMULACOES=15
PACOTE="adaptative_localization"      
LAUNCH="goal_pose_follow.launch" 

for ((i=1; i<=NUM_SIMULACOES; i++))
do
    echo "====================================================="
    echo " Iniciando Simulação $i de $NUM_SIMULACOES "
    echo "====================================================="

    # Executa o roslaunch normalmente. Ele vai parar sozinho graças ao required="true"
    roslaunch $PACOTE $LAUNCH

    echo "Simulação $i finalizada. Limpando processos..."
    killall -9 rosmaster gzserver gzclient rviz
    sleep 3
done

