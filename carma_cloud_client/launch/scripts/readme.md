sudo tcpdump -i lo -s0 -A dst port 33333

### After the tunnel is open, open a browser in local and type in: localhost:33333
It should display carma-cloud login page.

### get a process PID
sudo lsof -t -i:33333

### kill a process by using PID
sudo kill -9 $(sudo lsof -t -i:33333)

### shell into carma cloud EC2 server
ssh -i carma-cloud-test-1-1.pem ubuntu@www.carma-cloud.com

### after ssh into carma-cloud server, use below command to login as sudo user
sudo -i

### get a process PID
sudo lsof -t -i:10001

### kill a process by using PID
```sudo kill -9 $(sudo lsof -t -i:10001)```

### exit sudo user and exit  carma-cloud server
```exit``` 
```exit```

### open a forward tunnel to connect localhost to carma-cloud EC2 server via ports
```ssh -4 -f -i carma-cloud-test-1.pem -L 33333:localhost:8080 -N -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null ubuntu@www.carma-cloud.com```

### forward tunnel
sudo ./forward_tunnel.sh -u ubuntu -a www.carma-cloud.com -r 33333 -k carma-cloud-1.pem 

### open a reverse tunnel to connect carma-cloud EC2 server to localhost via ports
```sudo ./reverse_tunnel.sh -u ubuntu -a www.carma-cloud.com -r 22 -k carma-cloud-test-1.pem -p 10001```

### stop v2xhub app
sudo service tmxcore stop

### start v2xhub app
`sudo /usr/local/bin/tmxcore` (this will start V2X Hub in verbose for debugging, the terminal needs to stay open for V2X Hub to be accessible)


### At this point, V2X Hub GUI will be accessible - (username: v2xadmin, password: V2xHub#321)

## Automated scripts
Port Forwarding from source to destination.
```./open_tunnels.sh```

| Source Server   | Source Port  | Destination port | Destination Server|
| ------------- |:-------------:| -----:|-----:|
| V2xHub    | 33333 -->| 8080 | CARMA Cloud |
| CARMA Cloud        | 10001 -->      |   22222 | V2xHub|



