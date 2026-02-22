#expected at
#/usr/local/bin/update_dns.sh

#!/bin/bash
IP=$(hostname -I | awk '{print $1}')
CONF="/etc/dnsmasq.d/local.conf"

echo "address=/aesir.didi121.com/$IP" | sudo tee $CONF > /dev/null
echo "address=/subdomain.didi121.com/$IP" | sudo tee -a $CONF > /dev/null
sudo systemctl restart dnsmasq
