package org.firstinspires.ftc.teamcode.teamcode.Utilities.zLibraries.Files.Loggers;

import org.firstinspires.ftc.teamcode.teamcode.Utilities.zLibraries.Files.FileUtils.LogWriter;

import java.util.ArrayList;
import java.util.HashMap;

/**
 * More strictly-typed version of the GridLogger class.
 * If you're just trying to create a quick one-time CSV file, you're looking for GridLogger.
 * This is a really quick way to throw a lot of fatal errors.
 */
public class GridLoggerEx {

    private LogWriter writer;
    private ArrayList<String> columnHeaders = new ArrayList<>();;
    private HashMap<String, String> rowData = new HashMap<>();

    public GridLoggerEx(LogWriter writer){
        this.writer = writer;
    }

    /**
     * Adds a column header. Columns will be in order of calls to this function.
     * @param header
     */
    public void addHeader(String header){
        columnHeaders.add(header);
    }

    /**
     * Add a value to the logger under the specified column
     * @param column
     * @param entry
     */
    public void add(String column, String entry){
        rowData.put(column, entry);
    }

    /**
     * Add a value to the logger under the specified column
     * @param column
     * @param value
     */
    public void add(String column, double value){
        add(column, String.valueOf(value));
    }

    /**
     * Write a line of data to the log.
     * Returns a list of columns for which data was not found
     */
    public ArrayList<String> writeRow(){

        StringBuilder builder = new StringBuilder();

        ArrayList<String> failedColumns = new ArrayList<String>();

        // Initialize time

        builder = new StringBuilder();
        for (int i=0; i < columnHeaders.size(); i++){
            if(rowData.containsKey(columnHeaders.get(i))){
                builder.append(rowData.get(columnHeaders.get(i)));
            }else{
                failedColumns.add(columnHeaders.get(i));
            }
            if (i != columnHeaders.size() - 1) builder.append(",");
        }
        writer.writeLine(builder.toString());

        rowData.clear();
        return failedColumns;
    }

    public void writeHeaders(){
        StringBuilder builder = new StringBuilder();
        for (int i=0; i < columnHeaders.size(); i++){
            builder.append(columnHeaders.get(i));
            if (i != columnHeaders.size() - 1) builder.append(",");
        }
        writer.writeLine(builder.toString());
    }

    public void stop(){
        writer.stop();
    }

    public boolean isWriting(){
        return writer.isWriting();
    }
}