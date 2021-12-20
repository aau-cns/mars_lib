// Copyright (C) 2021 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#include "filesystem.h"

#include <sys/stat.h>
#include <sys/types.h>
#include <iostream>

namespace mars
{
filesystem::filesystem()
{
}

bool filesystem::IsDir(const std::string& name)
{
  struct stat info;

  if (stat(name.c_str(), &info) != 0)
  {
    return false;
  }
  else if (info.st_mode & S_IFDIR)  // Directory
  {
    return true;
  }

  return false;
}

bool filesystem::IsFile(const std::string& name)
{
  struct stat info;

  if (stat(name.c_str(), &info) != 0)
  {
    return false;
  }
  else if (info.st_mode & S_IFREG)  // Regular file
  {
    return true;
  }

  return false;
}

bool filesystem::MakeDir(const std::string& name)
{
  mode_t mode = 0755;

  if (IsDir(name))
  {
    std::cout << "[Info] Directory " << name << " already exists" << std::endl;
    return true;
  }
  else
  {
    int ret = mkdir(name.c_str(), mode);

    if (ret == 0)
    {
      return true;
    }
    else
    {
      return false;
    }
  }
}
}
