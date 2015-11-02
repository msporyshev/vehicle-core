#include "config_reader.h"

using namespace std;

template <typename T>
T config_reader::get_val(const map<string, map<string, T> > &m, const string &chapter, const string &name) const
{
    typename map<string, map<string, T> >::const_iterator j = m.find(chapter);
    if (j == m.end()) {
        cout << "illegal config file chapter " << chapter << endl;
        return T();
    }
    typename map<string, T>::const_iterator i = j->second.find(name);
    if (i == j->second.end()) {
        cout << "illegal config file name " << chapter << "." << name << endl;
        return T();
    }
    return i->second;
}

template <typename T>
bool config_reader::has_val(const map<string, map<string, T> > &m, const string &chapter, const string &name) const
{
    typename map<string, map<string, T> >::const_iterator j = m.find(chapter);
    if (j == m.end())
        return false;
    typename map<string, T>::const_iterator i = j->second.find(name);
    return i != j->second.end();
}

template string config_reader::get_val(const map<string, map<string, string> > &m, const string &chapter, const string &name) const;
template int config_reader::get_val(const map<string, map<string, int> > &m, const string &chapter, const string &name) const;
template double config_reader::get_val(const map<string, map<string, double> > &m, const string &chapter, const string &name) const;
template vector<double> config_reader::get_val(const map<string, map<string, vector<double> > > &m, const string &chapter, const string &name) const;

template bool config_reader::has_val(const map<string, map<string, string> > &m, const string &chapter, const string &name) const;
template bool config_reader::has_val(const map<string, map<string, int> > &m, const string &chapter, const string &name) const;
template bool config_reader::has_val(const map<string, map<string, double> > &m, const string &chapter, const string &name) const;
template bool config_reader::has_val(const map<string, map<string, vector<double> > > &m, const string &chapter, const string &name) const;

bool config_reader::set_double_val(const string &chapter, const string &name, double val)
{
    if (double_map.find(chapter) == double_map.end() ||
        double_map[chapter].find(name) == double_map[chapter].end()) {
        return false;
    }
    double_map[chapter][name] = val;
    return true;
}

bool config_reader::set_int_val(const string &chapter, const string &name, int val)
{
    if (int_map.find(chapter) == int_map.end() ||
        int_map[chapter].find(name) == int_map[chapter].end()) {
        return false;
    }
    int_map[chapter][name] = val;
    return true;
}

int config_reader::open(string filename)
{
    chapters_set.clear();
    chapters_vector.clear();
    ifstream f(filename.c_str());
    if (!f) {
        cout << "unable to find config file " << filename << endl;
        return 1;
    }
    string chapter = "";
    int i = 0;
    while (1) {
        string s;
        getline(f, s);
        i++;
        if (!f) break;
        if (s.find('#') != string::npos)
            s = s.substr(0, s.find('#'));
        istringstream ss(s);
        string name;
        ss >> name;
        if (!ss) continue;
        if (name[0] == '[') {
            chapter = name.substr(1, name.length()-2);
            if (chapters_set.find(chapter) != chapters_set.end())
                cout << "WARNING: duplicate chapter " << chapter << endl;
            chapters_set.insert(chapter);
            chapters_vector.push_back(chapter);
            continue;
        }
        string val;
        ss >> val;
        if (val == "=") ss >> val;
        if (!ss) {
            cout << "bad config file on line with " << name << endl;
            continue;
        }
        //определяем тип переменной: массив даблов, строка, целое число или дабл
        istringstream sss(val);
        if(val[0] == '{' && val[val.length() - 1] == '}') {
            double vec_dval;
            stringstream str;
            string num = "";
            for(size_t i = 1; i < val.length() - 1; i++) {
                if(val[i] != ',')
                    num += val[i];
                else {
                    str << num;
                    str >> vec_dval;
                    vector_double_map[chapter][name].push_back(vec_dval);
                    str.str("");
                    str.clear();
                    num = "";
                }
            }
            if(num.length()) {
                str << num;
                str >> vec_dval;
                vector_double_map[chapter][name].push_back(vec_dval);
                str.str("");
                str.clear();
                num = "";
            }
        } else if (val.find('.') != string::npos) {
            double dval; sss >> dval;
            double_map[chapter][name] = dval;
        } else {
            int ival; sss >> ival;
            if (!sss) {
                string_map[chapter][name] = val;
            } else {
                int_map[chapter][name] = ival;
            }
        }
    }
    return 0;
}

int config_reader::get_int_val(const string &chapter, const string &name) const
{
    return get_val<int>(int_map, chapter, name);
}

double config_reader::get_double_val(const string &chapter, const string &name) const
{
    return get_val<double>(double_map, chapter, name);
}

string config_reader::get_string_val(const string &chapter, const string &name) const
{
    return get_val<string>(string_map, chapter, name);
}

vector<double> config_reader::get_vector_double_val(const string &chapter, const string &name) const
{
    return get_val<vector<double> >(vector_double_map, chapter, name);
}

bool config_reader::has_int_val(const string &chapter, const string &name) const
{
    return has_val<int>(int_map, chapter, name);
}

bool config_reader::has_double_val(const string &chapter, const string &name) const
{
    return has_val<double>(double_map, chapter, name);
}

bool config_reader::has_string_val(const string &chapter, const string &name) const
{
    return has_val<string>(string_map, chapter, name);
}

bool config_reader::has_vector_double_val(const string &chapter, const string &name) const
{
    return has_val<vector<double> >(vector_double_map, chapter, name);
}

vector<string> config_reader::get_chapters_vector() const
{
    return chapters_vector;
}

template<typename T>
void config_reader::output_map(const map<string, map<string, T> > &m) const
{
    typename map<string, map<string, T> >::const_iterator i;
    for (i = m.begin(); i != m.end(); i++) {
        cout << "[" << i->first << "]" << endl;
        typename map<string, T>::const_iterator j;
        for (j = i->second.begin(); j != i->second.end(); j++) {
            cout << j->first << " = " << j->second << endl;
        }
    }
}

template void config_reader::output_map(const map<string, map<string, int> > &m) const;
template void config_reader::output_map(const map<string, map<string, double> > &m) const;
template void config_reader::output_map(const map<string, map<string, string> > &m) const;

template<>
void config_reader::output_map(const map<string, map<string, vector<double> > > &m) const
{
    map<string, map<string, vector<double> > >::const_iterator i;
    for (i = m.begin(); i != m.end(); i++) {
        cout << "[" << i->first << "]" << endl;
        map<string, vector<double> >::const_iterator j;
        for (j = i->second.begin(); j != i->second.end(); j++) {
            cout << j->first << " = {";
            for (size_t k = 0; k < j->second.size(); k++)
                cout << j->second[k] << ", ";
            cout << "}" << endl;
        }
    }
}

void config_reader::output_config() const
{
    cout << "***here is config state***" << endl;
    cout << endl << "***doubles***" << endl;
    output_map(double_map);
    cout << endl << "***ints***" << endl;
    output_map(int_map);
    cout << endl << "***strings***" << endl;
    output_map(string_map);
    cout << endl << "***vectors of double***" << endl;
    output_map(vector_double_map);
    cout << "***config state ends here***" << endl;
}
