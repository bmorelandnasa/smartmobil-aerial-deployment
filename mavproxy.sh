WINDOWS_IP=$(ip route show default | awk '{print $3}')
 
venv/bin/mavproxy.py \
  --master=udpin:0.0.0.0:14540 \
  --out=udp:${WINDOWS_IP}:14550 \
  --out=udp:127.0.0.1:14560 \
  --out=udp:${WINDOWS_IP}:14560
