#!/bin/bash
echo heartbeat > /sys/class/leds/a6281:blue:back/trigger                                                                       
echo heartbeat > /sys/class/leds/a6281:blue:front/trigger                                                                      
echo heartbeat > /sys/class/leds/a6281:blue:left/trigger                                                                       
echo heartbeat > /sys/class/leds/a6281:blue:right/trigger                                                                      
echo heartbeat > /sys/class/leds/a6281:green:back/trigger                                                                      
echo heartbeat > /sys/class/leds/a6281:green:front/trigger                                                                     
echo heartbeat > /sys/class/leds/a6281:green:left/trigger                                                                      
echo heartbeat > /sys/class/leds/a6281:green:right/trigger                                                                     
echo heartbeat > /sys/class/leds/a6281:red:back/trigger                                                                        
echo heartbeat > /sys/class/leds/a6281:red:front/trigger                                                                       
echo heartbeat > /sys/class/leds/a6281:red:left/trigger                                                                        
echo heartbeat > /sys/class/leds/a6281:red:right/trigger                                                                       
echo heartbeat > /sys/class/leds/cerebric:blue:status/trigger                                                                  
echo heartbeat > /sys/class/leds/cerebric:green:heartbeat/trigger                                                              
echo heartbeat > /sys/class/leds/cerebric:green:status/trigger                                                                 
echo heartbeat > /sys/class/leds/cerebric:red:busy/trigger                                                                     
echo heartbeat > /sys/class/leds/cerebric:red:status/trigger
