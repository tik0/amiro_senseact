#!/bin/bash
USR=itsowl

# Set proxy set/unset scripts
chown ${USR} /etc/environment
touch /etc/apt/apt.conf.d/01proxy
chown ${USR} /etc/apt/apt.conf.d/01proxy
sudo -H -u ${USR} bash -c "mkdir /home/${USR}/bin"
sudo -H -u ${USR} bash -c "touch /home/${USR}/bin/setProxy.sh && chmod 770 /home/${USR}/bin/setProxy.$
sudo -H -u ${USR} bash -c "touch /home/${USR}/bin/unsetProxy.sh && chmod 770 /home/${USR}/bin/unsetPr$

cat > /home/${USR}/bin/setProxy.sh <<EOF
#!/bin/bash
git config --global http.proxy http://claas\\skiba:Kazuk0099@172.20.20.10:8080
git config --global https.proxy https://claas\\skiba:Kazuk0099@172.20.20.10:8080
echo 'Acquire::http::Proxy "http://claas\skiba:Kazuk0099@172.20.20.10:8080";' > /etc/apt/apt.conf.d/0$
echo -e "\
PATH='/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games'\n\\
http_proxy=http://claas\\skiba:Kazuk0099@172.20.20.10:8080/\n\\
https_proxy=http://claas\\skiba:Kazuk0099@172.20.20.10:8080/\n\\
ftp_proxy=http://claas\\skiba:Kazuk0099@172.20.20.10:8080/\n\\
no_proxy='localhost,127.0.0.1,localaddress,.localdomain.com'\n\\
HTTP_PROXY=http://claas\\skiba:Kazuk0099@172.20.20.10:8080/\n\\
HTTPS_PROXY=http://claas\\skiba:Kazuk0099@172.20.20.10:8080/\n\\
FTP_PROXY=http://claas\\skiba:Kazuk0099@172.20.20.10:8080/\n\\
NO_PROXY='localhost,127.0.0.1,localaddress,.localdomain.com'\\
" > /etc/environment
EOF

cat > /home/${USR}/bin/unsetProxy.sh <<EOF
#!/bin/bash
git config --global --unset http.proxy
git config --global --unset https.proxy
echo '' > /etc/apt/apt.conf.d/01proxy
echo -e "\
PATH='/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games'\n\\
no_proxy='localhost,127.0.0.1,localaddress,.localdomain.com'\n\\
NO_PROXY='localhost,127.0.0.1,localaddress,.localdomain.com'\\
" > /etc/environment
EOF

echo "Source the .bashrc"
echo "You can now set or unset the proxy configurations with 'setProxy.sh' or 'unsetProxy.sh'"
echo "RESTART AFTER SETTING REQUIERD"
