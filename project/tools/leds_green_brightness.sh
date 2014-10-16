#!/bin/bash                                                                    
echo ${1} > /sys/class/leds/a6281:green:back/brightness                                                                      
echo ${1} > /sys/class/leds/a6281:green:front/brightness                                                                     
echo ${1} > /sys/class/leds/a6281:green:left/brightness                                                                      
echo ${1} > /sys/class/leds/a6281:green:right/brightness                                                                                                                                    
echo ${1} > /sys/class/leds/cerebric:green:heartbeat/brightness                                                              
echo ${1} > /sys/class/leds/cerebric:green:status/brightness                                                                 

