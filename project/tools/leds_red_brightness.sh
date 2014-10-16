#!/bin/bash                                                                  
echo ${1} > /sys/class/leds/a6281:red:back/brightness                                                                        
echo ${1} > /sys/class/leds/a6281:red:front/brightness                                                                       
echo ${1} > /sys/class/leds/a6281:red:left/brightness                                                                        
echo ${1} > /sys/class/leds/a6281:red:right/brightness                                                                                                                                    
echo ${1} > /sys/class/leds/cerebric:red:busy/brightness                                                                     
echo ${1} > /sys/class/leds/cerebric:red:status/brightness
