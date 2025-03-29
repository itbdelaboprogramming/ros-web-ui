CREATE DATABASE ROS_DB;

USE ROS_DB;

CREATE TABLE user(id int NOT NULL AUTO_INCREMENT PRIMARY KEY, username varchar(255) UNIQUE NOT NULL, email varchar(255) UNIQUE NOT NULL, full_name varchar(255) NOT NULL, password varchar(255) NOT NULL);

CREATE TABLE unit(id int NOT NULL AUTO_INCREMENT PRIMARY KEY, unit_name varchar(255) NOT NULL, user_id int NOT NULL, FOREIGN KEY (user_id) REFERENCES user(id), UNIQUE(unit_name, user_id));

CREATE TABLE pgm_data (id int NOT NULL AUTO_INCREMENT PRIMARY KEY, map_name varchar(100), file_size varchar(255), created_time datetime, modified_time datetime, IP_Address varchar(15), file_type varchar(255), data longtext, user_id int NOT NULL, FOREIGN KEY (user_id) REFERENCES user(id));

CREATE TABLE yaml_data(id int NOT NULL AUTO_INCREMENT PRIMARY KEY, name varchar(100), age int, email varchar(100), file_name varchar(255), file_size varchar(255), created_time datetime, modified_time datetime, author varchar(255), file_type varchar(255), data text, user_id int NOT NULL, FOREIGN KEY (user_id) REFERENCES user(id));
