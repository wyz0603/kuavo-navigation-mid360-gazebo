#!/bin/bash

IPs=""
for i in {2..254}; do
  IPs="$IPs 192.168.3.$i"
done

for i in {1..254}; do
  IPs="$IPs 10.10.10.$i"
done

for i in {1..254}; do
  IPs="$IPs 10.10.20.$i"
done

for i in {1..254}; do
  IPs="$IPs 10.0.8.$i"
done

for i in {1..254}; do
  IPs="$IPs 192.168.32.$i"
done

if [ ! -f /usr/local/bin/mkcert ]; then
  sudo apt install -y libnss3-tools
  curl -JLO "https://dl.filippo.io/mkcert/latest?for=linux/amd64"
  chmod +x mkcert-v*-linux-amd64
  sudo cp mkcert-v*-linux-amd64 /usr/local/bin/mkcert
fi

mkcert $IPs
mkcert -cert-file cert.pem -key-file key.pem $IPs
mkcert -CAROOT
cp ~/.local/share/mkcert/rootCA.pem .
mkdir -p ~/.tls
cp cert.pem ~/.tls
cp key.pem ~/.tls
