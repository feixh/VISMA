//
// Created by feixh on 10/25/17.
//
// unix
#include "dirent.h"
// stl
#include <iostream>
// I/O
#include "igl/readOBJ.h"
#include "igl/readPLY.h"
#include "json/json.h"

#include "vlslam.pb.h"
#include "utils.h"

namespace feh {

const std::string TermColor::red     = "\033[91m";
const std::string TermColor::green   = "\033[92m";
const std::string TermColor::blue    = "\033[94m";
const std::string TermColor::cyan    = "\033[96m";
const std::string TermColor::yellow  = "\033[93m";
const std::string TermColor::magenta = "\033[95m";
const std::string TermColor::gray    = "\033[90m";
const std::string TermColor::white   = "\033[97m";
const std::string TermColor::bold    = "\033[1m";
const std::string TermColor::end     = "\033[0m";
const std::string TermColor::endl     = "\033[0m\n";

std::ostream& operator<<(std::ostream& os, const Timer& obj)
{
    os << "....." << std::endl;
    // write obj to stream
    for ( auto it = obj.look_up_table_.begin(); it != obj.look_up_table_.end(); ++it ) {
        float elapsed = obj.report_average ? it->second / obj.counter_.at(it->first) : it->second;
        os << "[" << TermColor::bold+TermColor::green
           << obj.module_name_
           << TermColor::end << "]"
           << TermColor::cyan
           << it->first
           << TermColor::end
           << ":" << elapsed*1e-6 << " ms" << std::endl;
    }
    os << "....." << std::endl;
    return os;
}


bool Glob(const std::string &directory,
          const std::string &extension,
          std::vector<std::string> &filenames) {
    std::string suffix( (extension[0] == '.' ? "":".") + extension );
    std::string path(directory + "/");
    DIR *dir_ptr = opendir(path.c_str());
    struct dirent *entry;
    if (dir_ptr) {
        while ((entry = readdir(dir_ptr)) != NULL) {
            std::string entry_name(entry->d_name);
            if (entry_name.length() > suffix.length()
                && (entry_name.substr(entry_name.length() - suffix.length()).compare(suffix) == 0)) {
                filenames.push_back(entry_name.substr(0, entry_name.length() - suffix.length()));
            }
        }
        try {
            std::sort(filenames.begin(),
                      filenames.end(),
                      [](const std::string &a, const std::string &b) { return std::stof(a) < std::stof(b); });
        } catch (const std::invalid_argument &e) {
            std::sort(filenames.begin(),
                      filenames.end());
        }
        for (auto &filename: filenames) filename = path + filename + suffix;
        closedir(dir_ptr);
        return true;
    } else {
        return false;
    }
}

bool Glob(const std::string &directory,
          const std::string &extension,
          const std::string &prefix,
          std::vector<std::string> &filenames) {
    std::string suffix( (extension[0] == '.' ? "":".") + extension );
    std::string path(directory + "/");
    DIR *dir_ptr = opendir(path.c_str());
    struct dirent *entry;
    if (dir_ptr) {
        while ((entry = readdir(dir_ptr)) != NULL) {
            std::string entry_name(entry->d_name);
            if (entry_name.length() > suffix.length() + prefix.length()
                && (entry_name.substr(entry_name.length() - suffix.length()).compare(suffix) == 0)
                && (entry_name.substr(0, prefix.length()).compare(prefix) == 0)) {
                filenames.push_back(entry_name.substr(prefix.length(),
                                                      entry_name.length() - suffix.length() - prefix.length()));
            }
        }
        try {
            std::sort(filenames.begin(),
                      filenames.end(),
                      [](const std::string &a, const std::string &b) { return std::stof(a) < std::stof(b); });
        } catch (const std::invalid_argument &e) {
            std::sort(filenames.begin(),
                      filenames.end());
        }
        for (auto &filename: filenames) filename = path + prefix + filename + suffix;
        closedir(dir_ptr);
        return true;
    } else {
        return false;
    }
}

std::tuple<MatXf, MatXi> LoadMesh(const std::string &file) {
    MatXf V;
    MatXi F;
    if (!LoadMesh(file, V, F)) {
        throw MeshIO();
    } else {
        return std::make_tuple(V, F);
    }
}


bool LoadMesh(const std::string &file, MatXf &V, MatXi &F) {
    bool success = false;
    if (file.find(".obj") != std::string::npos) {
        success = igl::readOBJ(file, V, F);
    } else if (file.find(".ply") != std::string::npos) {
        success = igl::readPLY(file, V, F);
    }
    V = V.leftCols(3);
    F = F.leftCols(3);
    return success;
}



void MergeJson(Json::Value &a, const Json::Value &b) {
  if (!a.isObject() || !b.isObject()) return;
  for (const auto &key : b.getMemberNames()) 
    if (a[key].isObject()) 
      MergeJson(a[key], b[key]); 
    else 
      a[key] = b[key];
}

Json::Value LoadJson(const std::string &filename) {
    std::ifstream in(filename, std::ios::in);
    if (in.is_open()) {
        Json::Value out;
        in >> out;
        return out;
    } else {
        throw std::runtime_error(absl::StrFormat("failed to read file %s", filename));
    }
}

void SaveJson(const Json::Value &j, const std::string &filename) {
  std::ofstream ofs(filename, std::ios::out);
  assert(ofs.is_open());
  ofs << j;
  ofs.close();
}

}   // namespace feh

