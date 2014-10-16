#!/bin/bash
echo ${1} > /sys/class/leds/a6281:blue:back/brightness                                                                       
echo ${1} > /sys/class/leds/a6281:blue:front/brightness                                                                      
echo ${1} > /sys/class/leds/a6281:blue:left/brightness                                                                       
echo ${1} > /sys/class/leds/a6281:blue:right/brightness                                                                      
echo ${1} > /sys/class/leds/a6281:green:back/brightness                                                                      
echo ${1} > /sys/class/leds/a6281:green:front/brightness                                                                     
echo ${1} > /sys/class/leds/a6281:green:left/brightness                                                                      
echo ${1} > /sys/class/leds/a6281:green:right/brightness                                                                     
echo ${1} > /sys/class/leds/a6281:red:back/brightness                                                                        
echo ${1} > /sys/class/leds/a6281:red:front/brightness                                                                       
echo ${1} > /sys/class/leds/a6281:red:left/brightness                                                                        
echo ${1} > /sys/class/leds/a6281:red:right/brightness                                                                       
echo ${1} > /sys/class/leds/cerebric:blue:status/brightness                                                                  
echo ${1} > /sys/class/leds/cerebric:green:heartbeat/brightness                                                              
echo ${1} > /sys/class/leds/cerebric:green:status/brightness                                                                 
echo ${1} > /sys/class/leds/cerebric:red:busy/brightness                                                                     
echo ${1} > /sys/class/leds/cerebric:red:status/brightness
