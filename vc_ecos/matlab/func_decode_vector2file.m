function data_byte_len = func_decode_vector2file(fid,fid_config,format)

sop = fscanf(fid,'%d',4);
    fwrite(fid_config,sop,'uint32');
    
    dat = fscanf(fid,format,inf);
   
    for idx = 1:length(dat)-3
        if (format == '%d')
            fwrite(fid_config,dat(idx),'int32');
        else if (format == '%f')
            fwrite(fid_config,dat(idx),'double');
            else
            disp('format is not int32 and double');
            end
        end
    end
    data_byte_len    = sop(2);   
end