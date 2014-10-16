#!/bin/bash
echo ${1} > /sys/class/leds/a6281:blue:back/brightness                                                                       
echo ${1} > /sys/class/leds/a6281:blue:front/brightness                                                                      
echo ${1} > /sys/class/leds/a6281:blue:left/brightness                                                                       
echo ${1} > /sys/class/leds/a6281:blue:right/brightness                                                                                                                                          
echo ${1} > /sys/class/leds/cerebric:blue:status/brightness                                                                  
