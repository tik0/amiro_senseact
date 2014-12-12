clear all;

javaaddpath('/homes/tkorthals/RSBm/rsb-0.11-SNAPSHOT.jar')
javaaddpath('/homes/tkorthals/RSBm/rsb-matlab-0.11-SNAPSHOT.jar')
javaaddpath('/homes/tkorthals/RSBm/protobuf-java-2.5.0.jar')
javaaddpath('/homes/tkorthals/RSBm/rst-fleximon-0.11-SNAPSHOT.jar')
javaaddpath('/homes/tkorthals/RSBm/rst-sandbox-fleximon-0.11-SNAPSHOT.jar')

% rsb.matlab.ConverterRegistration.register('rst.generic.MethodCallType', 'MethodCall')

% b = rsb.matlab.ProtobufUtils.getBuilder('rst.generic.MethodCallType', 'MethodCall');
% b.setName(com.google.protobuf.ByteString.copyFromUtf8('doFoo'));
% a1 = b.addArgumentsBuilder();
% valueClass = rsb.matlab.ProtobufUtils.getInnerClass('rst.generic.ValueType', 'Value');
% typeClass = rsb.matlab.ProtobufUtils.getInnerClass(valueClass, 'Type');
% d = rsb.matlab.ProtobufUtils.getEnumConstant(typeClass, 'DOUBLE');
% a1.setType(d);
% a1.setDouble(12.34)

factory = rsb.Factory.getInstance();
info = factory.createInformer('/test');
info.activate();
pause(1)
info.send('example payload');
info.deactivate();