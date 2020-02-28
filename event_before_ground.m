function [value, isterminal, direction] = event_before_ground(t,x,h)
value = [x(5)>= 0.0001];
isterminal = 1;
direction = 0;
end