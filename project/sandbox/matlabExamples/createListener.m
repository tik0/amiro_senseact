function [] = createListener( scope, cb )
%CREATELISTENER Summary of this function goes here
%   Detailed explanation goes here
s = rsb.Scope(scope);
sub = rsb.Factory.getInstance().createListener(s);
sub.activate()
sub.addHandler(cb, true)
end

