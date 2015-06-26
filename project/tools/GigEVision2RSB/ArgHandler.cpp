/* 
 * File:   ArgHandler.cpp
 * Author: chris
 * 
 * Created on June 14, 2015, 5:29 PM
 */

#include "ArgHandler.hpp"
#include <exception>

ArgHandler::ArgHandler() {
}

bool ArgHandler::processArgs(int argc, char** argv) {
    for (int i = 1; i < argc; ++i) {
        string arg(argv[i]);

        if (isIdentifier(arg)) {
            if (isIdentifierSet && !isValueSet) {
                this->args.back().second = "";
            }
            this->args.push_back(make_pair(arg.substr(this->identifierPrefix.length()), ""));
            isIdentifierSet = true;
            isValueSet = false;

        } else {
            if (isIdentifierSet) {
                args.back().second = arg;
                isIdentifierSet = false;
                isValueSet = false;
            } else return false;
        }
    }
    if (isIdentifierSet && !isValueSet) this->args.back().second = "";
};

void ArgHandler::coutArgs() {
    cout << "Arguments" << endl << endl;
    for (int i = 0; i < args.size(); ++i) {
        cout << args[i].first << ": " << args[i].second << endl;
    }
    cout << endl << endl;
}

bool ArgHandler::isInteger(std::string s) {
    if (s.empty() || ((!isdigit(s[0])) && (s[0] != '-') && (s[0] != '+'))) return false;

    char * p;
    strtol(s.c_str(), &p, 10);

    return (*p == 0);
}

bool ArgHandler::isDouble(std::string s) {
    if (isInteger(s))return false;
    if (s.empty() || ((!isdigit(s[0])) && (s[0] != '-') && (s[0] != '+'))) return false;

    char * p;
    strtod(s.c_str(), &p);

    return (*p == 0);
}

bool ArgHandler::isBoolean(std::string s) {
    return (s == "true" || s == "false");
}

bool ArgHandler::isString(std::string s) {
    return !(isBoolean(s) || isInteger(s) || isDouble(s));
}

template<typename T> T ArgHandler::getValue(string ident) {
    string val = this->getValueAsString(ident);
    char * p;

    if (val == "true" || val == "false") {
        return toBoolean(val);
    } else if (isInteger(val)) {
        return toInteger(val);
    } else if (isDouble(val)) {
        return toDouble(val);
    } else {
        return val;
    }
}

bool ArgHandler::hasIdentifier(string ident) {
    for (vector<pair < string, string>>::iterator it = args.begin(); it != args.end(); ++it) {
        if (it->first == ident) {
            return true;
        }
    }
    return false;
}

string ArgHandler::getValueAsString(string ident) {
    for (vector<pair < string, string>>::iterator it = args.begin(); it != args.end(); ++it) {
        if (it->first == ident) {
            return it->second;
        }
    }
    return NULL;
}

bool ArgHandler::toBoolean(string ident) {
    if (ident == "true") {
        return true;
    } else {
        return false;
    }
}

double ArgHandler::toDouble(string val) {
    char* p;
    return strtod(val.c_str(), &p);

}

int ArgHandler::toInteger(string val) {
    char* p;
    return strtol(val.c_str(), &p, 10);
}

vector<pair<string, string>> ArgHandler::getValuesByPrefix(string prefix) {
    vector<pair<string, string>> rtn;
    for (std::vector<pair < string, string>>::iterator it = args.begin(); it != args.end(); ++it) {
        if (it->first.substr(0, prefix.length()) == prefix) {
            rtn.push_back(make_pair(it->first.substr(prefix.length()), it->second));
        }
    }
    return rtn;
}

bool ArgHandler::isIdentifier(string arg) {
    return (arg.substr(0, this->identifierPrefix.length()) == this->identifierPrefix);
}