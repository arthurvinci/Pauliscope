//
// Created by arthur on 29/03/2022.
//

#ifndef INSTANTIABLE_HPP
#define INSTANTIABLE_HPP

#define NOT_REAL_INSTANCE (4294967295U)

class Instantiable
{
public:
    explicit Instantiable(bool realInstance)
    {
        if(realInstance)
        {
            m_id = m_instances;
            m_instances++;
        }
        else
        {
            m_id = NOT_REAL_INSTANCE;
        }
    }

    static std::string genName(const std::string& instance_type, unsigned int id = m_instances+1) {
        std::ostringstream os;
        os << instance_type << id;
        std::string n = os.str();
        return n;
    };
protected:
    unsigned int m_id;
private:
    static unsigned int m_instances;
};

#endif //INSTANTIABLE_HPP
