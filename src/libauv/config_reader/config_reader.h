/*
 Шаблон класса config_reader применяется для чтения конфигурационных файлов на нижнем уровне.
 Файлы имеют следующий формат:

     [имя_раздела_1]
     переменная1 = значение1
     переменная2 = значение2
     ...

     [имя_раздела_2]
     ...

 Разделение проблеми желательно использовать как в примере.
 Имена переменных и разделов не должны содержать пробельных символов.
 Имена разделов не должны повторяться
 Если пара переменная-значение встречается без имени раздела,
 то имя раздела полагается равным пустой строке.
 Символ "#" задаёт комментарий до конца строки
 Знак "=" можно не использовать
 Тип переменной (целый или вещественный) определяется по наличию точки
 Массив заключается в фигурные скобки
 Если тип определить не удалось, то переменная считается строковой
*/

#ifndef CONFIG_READER_H
#define CONFIG_READER_H

#include <cctype>
#include <fstream>
#include <iostream>
#include <map>
#include <vector>
#include <set>
#include <sstream>
#include <string>

class config_reader
{
    std::map<std::string, std::map<std::string, int> > int_map;
    std::map<std::string, std::map<std::string, double> > double_map;
    std::map<std::string, std::map<std::string, std::string> > string_map;
    std::map<std::string, std::map<std::string, std::vector<double> > > vector_double_map;
    std::set<std::string> chapters_set;
    std::vector<std::string> chapters_vector;

    template <typename T>
    T get_val(const std::map<std::string, std::map<std::string, T> > &m, const std::string &chapter, const std::string &name) const;
    template <typename T>
    bool has_val(const std::map<std::string, std::map<std::string, T> > &m, const std::string &chapter, const std::string &name) const;
    template<typename T>
    void output_map(const std::map<std::string, std::map<std::string, T> > &m) const;

public:
    //0 - ok, иначе - ошибка
    int open(std::string filename);

    int get_int_val(const std::string &chapter, const std::string &name) const;
    double get_double_val(const std::string &chapter, const std::string &name) const;
    std::string get_string_val(const std::string &chapter, const std::string &name) const;
    std::vector<double> get_vector_double_val(const std::string &chapter, const std::string &name) const;

    bool set_int_val(const std::string &chapter, const std::string &name, int val);
    bool set_double_val(const std::string &chapter, const std::string &name, double val);

    bool has_int_val(const std::string &chapter, const std::string &name) const;
    bool has_double_val(const std::string &chapter, const std::string &name) const;
    bool has_string_val(const std::string &chapter, const std::string &name) const;
    bool has_vector_double_val(const std::string &chapter, const std::string &name) const;

    std::vector<std::string> get_chapters_vector() const;

    void output_config() const;
};

#endif
