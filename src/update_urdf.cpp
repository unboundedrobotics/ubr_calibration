/*
 * Copyright 2013 Unbounded Robotics Inc.
 * Author: Michael Ferguson
 */

#include <vector>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <ubr_calibration/update_urdf.h>

std::string updateURDF(const std::string &urdf, std::map<std::string, double> &offsets)
{
  TiXmlDocument xml_doc;
  xml_doc.Parse(urdf.c_str());

  TiXmlElement *robot_xml = xml_doc.FirstChildElement("robot");
  if (!robot_xml)
  {
    // TODO: error notification? We should never get here since URDF parse
    //       at beginning of calibration will fail
    return urdf;
  }

  /* Update each joint */
  for (TiXmlElement* joint_xml = robot_xml->FirstChildElement("joint"); joint_xml; joint_xml = joint_xml->NextSiblingElement("joint"))
  {
    const char * name = joint_xml->Attribute("name");

    /* Is there a joint calibration needed? */
    std::map<std::string, double>::iterator it = offsets.find(std::string(name));
    if (it != offsets.end())
    {
      double offset = offsets[std::string(name)];

      TiXmlElement *calibration_xml = joint_xml->FirstChildElement("calibration");
      if (calibration_xml)
      {
        /* Existing calibration, update rising attribute. */
        const char * rising_position_str = calibration_xml->Attribute("rising");
        if (rising_position_str != NULL)
        {
          try
          {
            offset += double(boost::lexical_cast<double>(rising_position_str));
            calibration_xml->SetDoubleAttribute("rising", offset);
          }
          catch (boost::bad_lexical_cast &e)
          {
            // TODO: error
          }
        }
        else
        {
          // TODO: error
        }
      }
      else
      {
        /* No calibration previously, add an element + attribute. */
        calibration_xml = new TiXmlElement("calibration");
        calibration_xml->SetDoubleAttribute("rising", offset);
        TiXmlNode * calibration = calibration_xml->Clone();
        joint_xml->InsertEndChild(*calibration);
      }
    }

    /* These consitute our origin. */
    std::vector<double> xyz(3, 0.0);
    std::vector<double> rpy(3, 0.0);

    /* Parse out if we have any updates */
    bool has_updates = false;
    it = offsets.find(std::string(name).append("_x"));
    if (it != offsets.end())
    {
      xyz[0] = it->second;
      has_updates = true;
    }
    it = offsets.find(std::string(name).append("_y"));
    if (it != offsets.end())
    {
      xyz[1] = it->second;
      has_updates = true;
    }
    it = offsets.find(std::string(name).append("_z"));
    if (it != offsets.end())
    {
      xyz[2] = it->second;
      has_updates = true;
    }
    it = offsets.find(std::string(name).append("_roll"));
    if (it != offsets.end())
    {
      rpy[0] = it->second;
      has_updates = true;
    }
    it = offsets.find(std::string(name).append("_pitch"));
    if (it != offsets.end())
    {
      rpy[1] = it->second;
      has_updates = true;
    }
    it = offsets.find(std::string(name).append("_yaw"));
    if (it != offsets.end())
    {
      rpy[2] = it->second;
      has_updates = true;
    }

    if (has_updates)
    {
      TiXmlElement *origin_xml = joint_xml->FirstChildElement("origin");
      if (origin_xml)
      {
        /* Update existing origin. */
        const char * xyz_str = origin_xml->Attribute("xyz");
        const char * rpy_str = origin_xml->Attribute("rpy");

        /* Split out xyz of origin, break into 3 strings. */
        std::vector<std::string> xyz_pieces;
        boost::split(xyz_pieces, xyz_str, boost::is_any_of(" "));

        if (xyz_pieces.size() == 3)
        {
          /* Merge xyz */
          for (int i = 0; i < 3; ++i)
          {
            double x = double(boost::lexical_cast<double>(xyz_pieces[i]) + xyz[i]);
            xyz_pieces[i] = boost::lexical_cast<std::string>(x);
          }
        }
        else
        {
          /* Create xyz */
          xyz_pieces.resize(3);
          xyz_pieces[0] = boost::lexical_cast<std::string>(xyz[0]);
          xyz_pieces[1] = boost::lexical_cast<std::string>(xyz[1]);
          xyz_pieces[2] = boost::lexical_cast<std::string>(xyz[2]);
        }

        /* Split out rpy of origin, break into 3 strings. */
        std::vector<std::string> rpy_pieces;
        boost::split(rpy_pieces, rpy_str, boost::is_any_of(" "));

        if (rpy_pieces.size() == 3)
        {
          /* Merge rpy */
          for (int i = 0; i < 3; ++i)
          {
            double x = double(boost::lexical_cast<double>(rpy_pieces[i]) + rpy[i]);
            rpy_pieces[i] = boost::lexical_cast<std::string>(x);
          }
        }
        else
        {
          /* Create rpy */
          rpy_pieces.resize(3);
          rpy_pieces[0] = boost::lexical_cast<std::string>(rpy[0]);
          rpy_pieces[1] = boost::lexical_cast<std::string>(rpy[1]);
          rpy_pieces[2] = boost::lexical_cast<std::string>(rpy[2]);
        }

        /* Update xml */
        origin_xml->SetAttribute("xyz", boost::join(xyz_pieces, " "));
        origin_xml->SetAttribute("rpy", boost::join(rpy_pieces, " "));
      }
      else
      {
        /* No existing origin, create an element with attributes. */
        origin_xml = new TiXmlElement("origin");
        std::vector<std::string> xyz_pieces(3);
        xyz_pieces[0] = boost::lexical_cast<std::string>(xyz[0]);
        xyz_pieces[1] = boost::lexical_cast<std::string>(xyz[1]);
        xyz_pieces[2] = boost::lexical_cast<std::string>(xyz[2]);
        origin_xml->SetAttribute("xyz", boost::join(xyz_pieces, " "));

        std::vector<std::string> rpy_pieces(3);
        rpy_pieces[0] = boost::lexical_cast<std::string>(rpy[0]);
        rpy_pieces[1] = boost::lexical_cast<std::string>(rpy[1]);
        rpy_pieces[2] = boost::lexical_cast<std::string>(rpy[2]);
        origin_xml->SetAttribute("rpy", boost::join(rpy_pieces, " "));

        TiXmlNode * origin = origin_xml->Clone();
        joint_xml->InsertEndChild(*origin);
      }
    }
  }

  /* Print to a string */
  TiXmlPrinter printer;
  printer.SetIndent("  ");
  xml_doc.Accept(&printer);
  std::string new_urdf = printer.CStr();

  return new_urdf;
}
