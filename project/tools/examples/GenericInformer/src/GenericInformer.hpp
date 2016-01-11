#ifndef SRC_GENERICINFORMER_HPP_
#define SRC_GENERICINFORMER_HPP_

// RSB
#include <rsb/Event.h>
#include <rsb/Factory.h>
#include <rsb/Handler.h>

#include <rsb/converter/Repository.h>
#include <rsb/converter/ProtocolBufferConverter.h>

// Boost
#include <boost/shared_ptr.hpp>

template <typename T>
class GenericInformer {
public:
	GenericInformer(const std::string& scope): outscope(scope) {
		boost::shared_ptr<rsb::converter::ProtocolBufferConverter<T>> converter(new rsb::converter::ProtocolBufferConverter<T>());
		rsb::converter::converterRepository<std::string>()->registerConverter(converter);
		rsb::Factory& factory = rsb::getFactory();
		informer = factory.createInformer<T> (outscope);
		msg = boost::make_shared<T>();
	}

	void publishMsg(){
		informer->publish(msg);
	}

	boost::shared_ptr<T> getMsgPointer(){
		return msg;
	}

private:
	std::string outscope;
	typename rsb::Informer<T>::Ptr informer;
	typename rsb::Informer<T>::DataPtr msg;
};

#endif /* SRC_GENERICINFORMER_HPP_ */
