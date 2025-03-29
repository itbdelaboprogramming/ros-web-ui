import os
import mysql.connector
from mysql.connector import Error
import yaml
import time
import pytz
from datetime import datetime
import math
import psutil
import base64
from config import *

# Function to read YAML file
def read_yaml_file(file_path):
    with open(file_path, 'r') as file:
        yaml_data = file.read()
    return yaml_data

# Function to read and possibly compress PGM file
def read_pgm_file(file_path):
    # binary pgm data is b64 encoded, please b64 decode to save the file in frontend
    with open(file_path, 'rb') as pgm_file:
        pgm_content = pgm_file.read()
        base64_encoded = base64.b64encode(pgm_content).decode('utf-8')
    return base64_encoded

def format_time_to_jst(local_time_string):
    # First, parse the local time string into a datetime object
    local_time = datetime.strptime(local_time_string, '%Y-%m-%d %H:%M:%S')

    # Localize the datetime object to the local time zone
    local_timezone = pytz.timezone(LOCAL_TIMEZONE)
    local_dt_with_tz = local_timezone.localize(local_time, is_dst=None)

    # Convert the time
    jst_timezone = pytz.timezone(CONVERT_TIMEZONE)
    jst_dt = local_dt_with_tz.astimezone(jst_timezone)

    return jst_dt.strftime('%Y-%m-%d %H:%M:%S')

def get_file_metadata(file_path, file_type):
    stats = os.stat(file_path)  # Define stats here
    file_size = format_file_size(stats.st_size)  # File size with units
    created_time = format_time_to_jst(time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(stats.st_ctime)))  # Creation time
    modified_time = format_time_to_jst(time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(stats.st_mtime)))  # Modification time
    author = ''

    return file_size, created_time, modified_time, author, file_type

def format_file_size(size_bytes):
    if size_bytes == 0:
        return "0B"
    size_name = ("B", "KB", "MB", "GB", "TB")
    i = int(math.floor(math.log(size_bytes, 1024)))
    p = math.pow(1024, i)
    s = round(size_bytes / p, 2)
    return f"{s} {size_name[i]}"

# SETIAP MULAI TERMINAL YAHHHH JANGAN SAMPE KEDOBEL SEMUA JADI PAKE INI
# GATAU BISA ATAU NGGA

def file_exists_in_database(file_name, table_name):
    connection = None  # Initialize connection variable
    try:
        connection = mysql.connector.connect(
            host=MYSQL_HOST,
            database=MYSQL_DATABASE,
            user=MYSQL_USER,
            password=MYSQL_PASS
        )
        cursor = connection.cursor()

        column_name = "file_name" if table_name == "yaml_data" else "map_name"
        sql_query = f"SELECT EXISTS(SELECT 1 FROM {table_name} WHERE {column_name} = %s)"
        cursor.execute(sql_query, (file_name,))
        result = cursor.fetchone()

        return result[0] == 1

    except Error as e:
        print(f"Error checking file in database: {e}")
        return False

    finally:
        if connection and connection.is_connected():
            cursor.close()
            connection.close()


def get_local_ip():
    try:
        for net, data in psutil.net_if_addrs().items():
            if net == 'zt5u4t7gdz':  # Specify the ZeroTier network interface name
                for item in data:
                    if ":" not in item.address:  # Look for an IPv4 address
                        return item.address
        return '127.0.0.1'  # Fallback to localhost if ZeroTier IP is not found
    except Exception as e:
        print(f"Error getting local IP: {e}")
        return '127.0.0.1'


# Function to insert YAML data into MySQL
def insert_yaml_data_into_mysql(yaml_data, file_name):
    file_path = os.path.join(DIRECTORY_TO_WATCH, file_name)
    file_size, created_time, modified_time, author, file_type = get_file_metadata(file_path, "yaml")

    if not file_exists_in_database(file_name, "yaml_data"):
        try:
            connection = mysql.connector.connect(
                host=MYSQL_HOST,
                database=MYSQL_DATABASE,
                user=MYSQL_USER,
                password=MYSQL_PASS
            )
            cursor = connection.cursor()

            insert_single_yaml_record(cursor, file_name, file_size, created_time, modified_time, author, file_type, yaml_data)

            connection.commit()
            print(f"YAML data inserted successfully for {file_name}")

        except Error as e:
            print(f"Error inserting YAML data for {file_name}: {e}")

        finally:
            if connection and connection.is_connected():
                cursor.close()
                connection.close()

def insert_single_yaml_record(cursor, file_name, file_size, created_time, modified_time, author, file_type, yaml_data):
    parsed_list = file_name.split('_')
    user_id = 0
    if parsed_list:
        user_id = parsed_list[0]
    sql_query = """
        INSERT INTO yaml_data (
            file_name, file_size, created_time, modified_time, author, file_type, data, user_id
        ) VALUES (%s, %s, %s, %s, %s, %s, %s, %s)
    """
    cursor.execute(sql_query, (
        file_name, file_size, created_time, modified_time, author, file_type, yaml_data, user_id
    ))


# Function to insert PGM data into MySQL
def insert_pgm_data_into_mysql(pgm_data, file_name):
    file_path = os.path.join(DIRECTORY_TO_WATCH, file_name)  # Derive file_path from file_name
    file_size, created_time, modified_time, author, file_type = get_file_metadata(file_path, "pgm")
    local_ip = get_local_ip()

    connection = None  # Initialize connection here

    if not file_exists_in_database(file_name, "pgm_data"):
        try:
            connection = mysql.connector.connect(
                host=MYSQL_HOST,
                database=MYSQL_DATABASE,
                user=MYSQL_USER,
                password=MYSQL_PASS
            )

            cursor = connection.cursor()

            # Generate a new ID every hour
            current_hour = datetime.now().strftime('%Y-%m-%d %H')
            last_inserted_hour = get_last_inserted_hour(cursor, "pgm_data")

            # Check if the current hour is greater than the last inserted hour
            if current_hour > last_inserted_hour:
                reset_id_sequence(cursor, "pgm_data")
                print("ID sequence reset for pgm_data")

            # Get the next available ID
            next_id = get_next_id(cursor, "pgm_data")

            parsed_list = file_name.split('_')
            user_id = 0
            if parsed_list:
                user_id = parsed_list[0]

            # Prepare data for insertion
            sql_query = """
            INSERT INTO pgm_data (
                id, map_name, file_size, created_time, modified_time, IP_Address, file_type, data, user_id
            ) VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s)
            """
            cursor.execute(sql_query, (
                next_id, file_name, file_size, created_time, modified_time, local_ip, file_type, pgm_data, user_id
            ))

            connection.commit()
            print(f"PGM data inserted successfully for {file_name}")

        except Error as e:
            print(f"Error inserting PGM data for {file_name}: {e}")

        finally:
            if connection and connection.is_connected():
                cursor.close()
                connection.close()

def get_next_id(cursor, table_name):
    sql_query = f"SELECT MAX(id) FROM {table_name}"
    cursor.execute(sql_query)
    result = cursor.fetchone()
    if result and result[0]:
        return result[0] + 1
    return 1  # If no records exist, start from 1

# BUAT ID ID AN INI ALGORITMA SEN GANTI SESUAI KEBUTUHAN

def get_last_inserted_hour(cursor, table_name):
    sql_query = f"SELECT MAX(created_time) FROM {table_name}"
    cursor.execute(sql_query)
    result = cursor.fetchone()
    if result and result[0]:
        last_inserted_hour = result[0].strftime('%Y-%m-%d %H')
        return last_inserted_hour
    return ''  # Return an empty string if no records exist

def reset_id_sequence(cursor, table_name):
    sql_query = f"ALTER TABLE {table_name} AUTO_INCREMENT = 1"
    cursor.execute(sql_query)

# Algoritma sederhana within one hour changes ID

# Rename yahhhhhhhhh
def update_file_in_database(old_file_name, new_file_name, table_name):
    try:
        connection = mysql.connector.connect(
            host=MYSQL_HOST,
            database=MYSQL_DATABASE,
            user=MYSQL_USER,
            password=MYSQL_PASS
        )
        cursor = connection.cursor()

        # Get current time in JST for modified_time update
        current_time = format_time_to_jst(datetime.now().strftime('%Y-%m-%d %H:%M:%S'))

        if table_name == "pgm_data":
            sql_query = """
            UPDATE pgm_data
            SET map_name = %s, modified_time = %s
            WHERE map_name = %s
            """
        else:
            sql_query = """
            UPDATE yaml_data
            SET file_name = %s, modified_time = %s
            WHERE file_name = %s
            """

        params = (new_file_name, current_time, old_file_name)
        cursor.execute(sql_query, params)


        connection.commit()
        print(f"Database updated: {old_file_name} to {new_file_name} in {table_name}")

    except Error as e:
        print(f"Error updating file in database: {e}")

    finally:
        if connection and connection.is_connected():
            cursor.close()
            connection.close()


# HAPUS HAPUS YAHHHHHHHHHHHH
def delete_file_from_database(file_name, table_name):
    try:
        connection = mysql.connector.connect(
            host=MYSQL_HOST,
            database=MYSQL_DATABASE,
            user=MYSQL_USER,
            password=MYSQL_PASS
        )
        cursor = connection.cursor()

        column_name = "file_name"
        if table_name == "pgm_data":
            column_name = "map_name"

        sql_query = f"DELETE FROM {table_name} WHERE {column_name} = %s"
        cursor.execute(sql_query, (file_name,))

        connection.commit()
        print(f"Database entry deleted for {file_name} in {table_name}")

    except Error as e:
        print(f"Error deleting file from database: {e}")

    finally:
        if connection and connection.is_connected():
            cursor.close()
            connection.close()


# Iterate through files in the folder
for filename in os.listdir(DIRECTORY_TO_WATCH):
    file_path = os.path.join(DIRECTORY_TO_WATCH, filename)

    if filename.endswith(".yaml"):
        yaml_data = read_yaml_file(file_path)
        insert_yaml_data_into_mysql(yaml_data, filename)

    elif filename.endswith(".pgm"):
        pgm_data = read_pgm_file(file_path)
        insert_pgm_data_into_mysql(pgm_data, filename)

print("Process completed.")