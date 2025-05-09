import time
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler
from config import *
import os
import mysql.connector
from mysql.connector import Error
import threading
from sync_from_database import sync_database_to_local
from import_data_to_mysql import (
    read_yaml_file,
    read_pgm_file,
    insert_yaml_data_into_mysql,
    insert_pgm_data_into_mysql,
    update_file_in_database,
    delete_file_from_database
)

class Watcher:

    def __init__(self):
        self.observer = Observer()
        self.directory = DIRECTORY_TO_WATCH
        
        self.sync_interval = 3
        
        # Logging the system
        print("Monitoring directory:", self.directory, flush=True)
        print(f"Database Host: {MYSQL_HOST}")
        print(f"Database Name: {MYSQL_DATABASE}")
        print(f"Database User: {MYSQL_USER}")
        print(f"Local Timezone: {LOCAL_TIMEZONE}")
        print(f"Convert Timezone: {CONVERT_TIMEZONE}")
        print("Starting to monitor files...")
        print("=========================================")
        
    def schedule_sync(self):
        while True:
            time.sleep(self.sync_interval)
            try:
                sync_database_to_local()
                print("Periodic sync from database completed.")
            except Exception as e:
                print(f"Periodic sync error: {e}")

    def run(self):
        # Process existing files before starting the observer
        process_existing_files(self.directory)
        
        sync_thread = threading.Thread(target=self.schedule_sync)
        sync_thread.start()
        
        event_handler = Handler()
        self.observer.schedule(event_handler, self.directory, recursive=True)
        self.observer.start()
        try:
            while True:
                time.sleep(5)
        except KeyboardInterrupt:
            self.observer.stop()
            print("Observer Stopped")

        self.observer.join()

def file_exists_in_database(file_name, table_name):
    connection = None
    try:
        connection = mysql.connector.connect(
            host=MYSQL_HOST,
            port=MYSQL_PORT,  # e.g., 3306 or your custom port
            database=MYSQL_DATABASE,
            user=MYSQL_USER,
            password=MYSQL_PASS,
            ssl_ca=MYSQL_SSL_CA,         # path to your CA certificate file
            ssl_verify_cert=True,        # ensures the server certificate is verified
            connection_timeout=10        # optional: set a timeout to avoid hanging
        )
        cursor = connection.cursor()

        column_name = "file_name"
        if table_name == "pgm_data":
            column_name = "map_name"

        sql_query = f"SELECT COUNT(*) FROM {table_name} WHERE {column_name} = %s"
        cursor.execute(sql_query, (file_name,))
        result = cursor.fetchone()[0]

        return result > 0

    except Error as e:
        print(f"Error checking if file exists in database: {e}")
        return False

    finally:
        if connection and connection.is_connected():
            cursor.close()
            connection.close()

# YANG UDAH ADA
def process_existing_files(folder_path):
    # Logging the system
    print("Processing existing files in the directory...")
    print("=========================================")
    for filename in os.listdir(folder_path):
        file_path = os.path.join(folder_path, filename)

        if filename.endswith(".yaml"):
            yaml_data = read_yaml_file(file_path)
            insert_yaml_data_into_mysql(yaml_data, filename)

        elif filename.endswith(".pgm"):
            pgm_data = read_pgm_file(file_path)
            insert_pgm_data_into_mysql(pgm_data, filename)
    print("=========================================")

class Handler(FileSystemEventHandler):

    @staticmethod
    def on_created(event):
        if event.is_directory:
            return None

        file_name = os.path.basename(event.src_path)

        if file_name.endswith(".yaml"):
            try:
                if not file_exists_in_database(file_name, "yaml_data"):
                    yaml_data = read_yaml_file(event.src_path)
                    insert_yaml_data_into_mysql(yaml_data, file_name)
                    print(f"YAML data inserted for {file_name}")
                else:
                    print(f"File {file_name} already exists in the database.")
            except Exception as e:
                print(f"Error processing YAML file {event.src_path}: {e}")

        elif file_name.endswith(".pgm"):
            try:
                if not file_exists_in_database(file_name, "pgm_data"):
                    pgm_data = read_pgm_file(event.src_path)
                    insert_pgm_data_into_mysql(pgm_data, file_name)
                    print(f"PGM data inserted for {file_name}")
                else:
                    print(f"File {file_name} already exists in the database.")
            except Exception as e:
                print(f"Error processing PGM file {event.src_path}: {e}")

    @staticmethod
    def on_moved(event):
        if event.is_directory:
            return None

        old_file_name = os.path.basename(event.src_path)
        new_file_name = os.path.basename(event.dest_path)

        if old_file_name.endswith(".yaml") or old_file_name.endswith(".pgm"):
            table_name = "yaml_data" if old_file_name.endswith(".yaml") else "pgm_data"
            if not file_exists_in_database(new_file_name, table_name):
                update_file_in_database(old_file_name, new_file_name, table_name)

    @staticmethod
    def on_deleted(event):
        if event.is_directory:
            return None

        file_name = os.path.basename(event.src_path)

        if file_name.endswith(".yaml") or file_name.endswith(".pgm"):
            table_name = "yaml_data" if file_name.endswith(".yaml") else "pgm_data"
            if file_exists_in_database(file_name, table_name):
                delete_file_from_database(file_name, table_name)

    @staticmethod
    def on_any_event(event):
        if event.event_type == 'created':
            Handler.on_created(event)
        elif event.event_type == 'moved':
            Handler.on_moved(event)
        elif event.event_type == 'deleted':
            Handler.on_deleted(event)

if __name__ == '__main__':
    w = Watcher()
    w.run()