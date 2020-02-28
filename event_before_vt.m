function [value, isterminal, direction] = event_before_vt(t,x,v_t)
value = [x(6) - v_t >= 0.0001];
isterminal = 1;
direction = 0;
end