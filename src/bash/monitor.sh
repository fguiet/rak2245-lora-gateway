#!/bin/bash
#
# 2020-07-22
# Inspired from https://github.com/disk91/ttnMonitorGateway and https://www.disk91.com/2020/technology/lora/monitor-a-gateway-connected-to-the-things-network/
#
# This script is used to check gateway status from time to time
# It restarts 'chirpstack-packet-multiplexer'  service if needed (maybe the origin of the failure)
# It sends a http request to NodeRED flow => will send a SMS
# Require : bc (sudo apt-get install bc)
#
#
cd `dirname $0`

TTNCLI=ttnctl-linux-arm

# Configure the gatweays to be monitored
# Format
# ttn_gateway_is, name diplayed on webhook, Monitoring ON/OFF, last gateway state (OK at start)
gatewaysConf=$(cat <<-END
eui-dca632fffe365d9c,RAK2245-Gateway,ON,OK@
END
)

# tmp file to save the gateway state
TMPFILE=LoRaWanGwMonitor.tmp
LOGFILE=LoRaWanGwLog.log

# timeout to detect disconnection
TIMEOUT_S=300

#
# as a parameter $1, the TTN gateway id
function checkOneGateway {
  delta=0
  d=`./${TTNCLI} gateways status $1 | grep "Last seen" | tr -s " " | cut -d " " -f 4,5,7 | sed "s/^\(.*\)\.\(.*\) \(.*\)$/\1 \3/"`
  if [ -z "$d" ] ; then
   # sometime the date is invalid from the API
   return 2
  fi 

  t=`date --date="${d}" +"%s"`
  now=`date "+%s"`
  delta=`echo "$now - $t" | bc `
  if [ $delta -gt $TIMEOUT_S ] ; then
     return 1
  fi
  return 0
}

#
# Update the gwLastState for the gwId $1 with the new state $2
function changeGatewayStatus {
  gateways=`echo $gateways | sed -e "s/\(^.*$1,[^,]\+,[^,]\+,\)[^@]\+\(@.*$\)/\1$2\2/"`
}

#
# Fire the NodeRED Alert
# Param 1 : gwName
# Param 2 : gwLastState
function fireNodeRED {
   curl -X GET "http://nodered.guiet.lan/publish-lorawan-status?gatewayid=$1&status=$2"
}

function main {
  if [ -f $TMPFILE ] ; then
    gateways="`cat $TMPFILE`"
  else
    gateways="$gatewaysConf" 
  fi

  for gw in $(echo $gateways | sed "s/@/ /g"); do
    gwId=`echo $gw | cut -d "," -f 1`
    gwName=`echo $gw | cut -d "," -f 2`
    gwMonitor=`echo $gw | cut -d "," -f 3`
    gwLastState=`echo $gw | cut -d "," -f 4`

    echo ">> $(date '+%Y-%m-%d %H:%M:%S') - Checking : $gwId $gwName $gwMonitor $gwLastState ($gw)" >> $LOGFILE
    if [ $gwMonitor == "ON" ] ; then
      if checkOneGateway $gwId ; then
        if [ $gwLastState == "KO" ] ; then
          changeGatewayStatus $gwId "OK"
 	  echo "Gateway $gwName is back online on `date`">> $LOGFILE
	  fireNodeRED $gwName "online"
        fi
      else
        if [ $? -eq 1 ] ; then 
          if [ $gwLastState == "OK" ] ; then
            changeGatewayStatus $gwId "KO"
	    echo "gateway $gwName has stopped on `date`">> $LOGFILE
	    echo "Restarting chirpstack-packet-multiplexer">> $LOGFILE
            sudo systemctl restart chirpstack-packet-multiplexer 2>&1 >> $LOGFILE
            echo "chirpstack-packet-multiplexer restarted">> $LOGFILE 
	    fireNodeRED $gwName "offline"
          fi
        fi
      fi
    fi
  done 
  echo $gateways > $TMPFILE
}

main
