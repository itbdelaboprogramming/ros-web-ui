import os
import mysql.connector
from mysql.connector import Error
from config import *
import base64
import yaml

def download_pgm_from_database(map_name, data):
    file_path = os.path.join(DIRECTORY_TO_WATCH, map_name)
    pgm_content = base64.b64decode(data)
    with open(file_path, 'wb') as pgm_file:
        pgm_file.write(pgm_content)
    print(f"Downloaded PGM: {map_name}")

def download_yaml_from_database(file_name, data):
    file_path = os.path.join(DIRECTORY_TO_WATCH, file_name)
    with open(file_path, 'w') as yaml_file:
        yaml_file.write(data)
    print(f"Downloaded YAML: {file_name}")

def sync_database_to_local():
    try:
        connection = mysql.connector.connect(
            host=MYSQL_HOST,
            port=MYSQL_PORT,
            database=MYSQL_DATABASE,
            user=MYSQL_USER,
            password=MYSQL_PASS,
            ssl_ca=MYSQL_SSL_CA,
            ssl_verify_cert=True
        )
        cursor = connection.cursor(dictionary=True)

        # Check YAML files
        cursor.execute("SELECT file_name, data FROM yaml_data")
        yaml_records = cursor.fetchall()
        for record in yaml_records:
            local_path = os.path.join(DIRECTORY_TO_WATCH, record['file_name'])
            if not os.path.exists(local_path):
                download_yaml_from_database(record['file_name'], record['data'])

        # Check PGM files
        cursor.execute("SELECT map_name, data FROM pgm_data")
        pgm_records = cursor.fetchall()
        for record in pgm_records:
            local_path = os.path.join(DIRECTORY_TO_WATCH, record['map_name'])
            if not os.path.exists(local_path):
                download_pgm_from_database(record['map_name'], record['data'])

    except Error as e:
        print(f"Error syncing from database: {e}")
    finally:
        if connection.is_connected():
            cursor.close()
            connection.close()

if __name__ == "__main__":
    sync_database_to_local()