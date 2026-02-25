FROM ros/armv7:humble-gst-cv

ENV DEBIAN_FRONTEND=noninteractive

# Copia i file necessari
COPY Desktop/workspace/ /workspace

# Imposta la directory di lavoro
WORKDIR /workspace

# Esegui la build del workspace con colcon

#RUN colcon build

# Copia il file di inizializzazione 
COPY ros_entrypoint.sh /ros_entrypoint.sh 

RUN chmod +x /ros_entrypoint.sh 

# Imposta il comando di avvio 

ENTRYPOINT ["/ros_entrypoint.sh"]

# Comando di default per avviare il launch file 

#CMD ["ros2", "run", "ras_pi_publisher", "ras_pi_publisher"]

CMD ["/bin/bash"]
