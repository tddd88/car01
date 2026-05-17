# Deployment Instructions

## bringup_auto.service

To install and enable auto-launch on boot:
```bash
sudo cp /home/ubuntu/car01/deploy/bringup_auto.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable bringup_auto.service
sudo systemctl start bringup_auto.service
```
