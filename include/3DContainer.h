/*
 * 3DContainer.h
 *
 *  Created on: Sep 8, 2017
 *      Author: mekya
 */

#ifndef THREE_D_DCONTAINER_H_
#define THREE_D_DCONTAINER_H_

#include "draco/compression/encode.h"
#include "draco/compression/decode.h"
#include "draco/io/ply_decoder.h"
#include "draco/io/ply_encoder.h"
#include "draco/io/obj_encoder.h"
#include "draco/io/obj_decoder.h"
#include "draco/core/cycle_timer.h"
#include "draco/io/mesh_io.h"
#include "draco/io/point_cloud_io.h"
#include <iostream>
#include <fstream>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
}


using namespace std;
using namespace draco;

const int OBJ_FORMAT = 0;
const int PLY_FORMAT = 1;

//const int STREAM_INDEX_MESH = 0;
const int STREAM_INDEX_MESH_TEXTURE = 0;
//const int STREAM_INDEX_SPACE_CONTAINER_MESH = 2;
const int STREAM_INDEX_SPACE_CONTAINER_TEXTURE = 1;




class ThreeDContainer {

public:

	ThreeDContainer() {
		av_register_all();
		format_context = NULL;

		/*
		 * is_point_cloud(false),
      --pos_quantization_bits(14),
      --tex_coords_quantization_bits(12),
      tex_coords_deleted(false),
      --normals_quantization_bits(10),
      normals_deleted(false),
      generic_quantization_bits(8),
      generic_deleted(false),
      compression_level(7),
      use_metadata(false)
		 */



		encoder.SetAttributeQuantization(draco::GeometryAttribute::POSITION,
				14);
		encoder.SetAttributeQuantization(draco::GeometryAttribute::TEX_COORD,
				12);

		encoder.SetAttributeQuantization(draco::GeometryAttribute::NORMAL,
				10);

		encoder.SetAttributeQuantization(draco::GeometryAttribute::GENERIC,
				8);


		const int speed = 10 - 0;
		encoder.SetSpeedOptions(speed, speed);
	}

	/**
	 * Opens file for reading or writing
	 *
	 * @param filename
	 * The name of the file to read or to write
	 *
	 * @param isWriteMode
	 * true if file is going to be written,
	 * false if file is going to be read
	 *
	 * @return bool
	 * true if it opens file successfully otherwise it returns false
	 */
	bool open(char* filename, bool isWriteMode) {
		if (isWriteMode) {
			return openForWrite(filename);
		}
		else {
			return openForRead(filename);
		}
	}

	/**
	 * Opens file for reading
	 *
	 * @param filename
	 * The name of the file to read
	 *
	 * @return bool
	 * true if it opens file successfully otherwise it returns false
	 */
	bool openForRead(char* filename) {
		this->isWriteMode = false;
		int ret = avformat_open_input(&format_context, filename, 0, 0);
		if (ret < 0) {
			return false;
		}
		return true;

	}

	/**
	 * Opens file for write
	 *
	 * @param filename
	 * The name of the file to write
	 *
	 * @return
	 * true if it opens file successfully otherwise it returns false
	 */
	bool openForWrite(char* filename) {
		this->isWriteMode = true;
		int ret;
		ret = avformat_alloc_output_context2(&format_context, NULL, NULL, filename);
		if (ret < 0) {
			cout << "Cannot allocate output context";
			return false;
		}



		{

			//Create stream for texture files - stream index will be 1
			out_stream_texture = avformat_new_stream(format_context, NULL);
			if (!out_stream_texture) {
				cout << "Cannot add new stream";
				return false;
			}

			AVCodec* codec = avcodec_find_encoder(AV_CODEC_ID_PNG);
			if (!codec) {
				cout << "Cannot find encoder";
				return false;
			}
			enc_ctx_texture =  avcodec_alloc_context3(codec); //out_stream->codec; //
			if (!enc_ctx_texture) {
				cout << "Cannot allocate for encoder context";
				return false;
			}

			enc_ctx_texture->height = 4320;
			enc_ctx_texture->width = 7680;
			AVRational aspect_ratio;
			aspect_ratio.num = 16;
			aspect_ratio.den = 9;
			enc_ctx_texture->sample_aspect_ratio = aspect_ratio;
			enc_ctx_texture->pix_fmt = AV_PIX_FMT_RGB24;
			/* video time_base can be set to whatever is handy and supported by encoder */
			AVRational time_base;
			time_base.num = 1;
			time_base.den = 25;
			enc_ctx_texture->time_base = time_base;

			ret = avcodec_open2(enc_ctx_texture, codec, NULL);
			if (ret < 0) {
				char err[4096];
				av_strerror(ret, err, sizeof(err));
				cout << "Cannot open video encoder for stream. error def: " << err <<endl;
				return false;
			}

			avcodec_parameters_from_context(out_stream_texture->codecpar, enc_ctx_texture);

			cout << " stream index for texture files: " << out_stream_texture->index << std::endl;

		}



		{

			//Create stream for space container texture files - stream index will be 3
			out_stream_space_texture = avformat_new_stream(format_context, NULL);
			if (!out_stream_space_texture) {
				cout << "Cannot add new stream";
				return false;
			}

			AVCodec* codec = avcodec_find_encoder(AV_CODEC_ID_PNG);
			if (!codec) {
				cout << "Cannot find encoder";
				return false;
			}
			enc_ctx_space_texture =  avcodec_alloc_context3(codec); //out_stream->codec; //
			if (!enc_ctx_space_texture) {
				cout << "Cannot allocate for encoder context";
				return false;
			}

			enc_ctx_space_texture->height = 4320;
			enc_ctx_space_texture->width = 7680;
			AVRational aspect_ratio;
			aspect_ratio.num = 16;
			aspect_ratio.den = 9;
			enc_ctx_space_texture->sample_aspect_ratio = aspect_ratio;
			enc_ctx_space_texture->pix_fmt = AV_PIX_FMT_RGB24;
			/* video time_base can be set to whatever is handy and supported by encoder */
			AVRational time_base;
			time_base.num = 1;
			time_base.den = 25;
			enc_ctx_space_texture->time_base = time_base;

			ret = avcodec_open2(enc_ctx_space_texture, codec, NULL);
			if (ret < 0) {
				char err[4096];
				av_strerror(ret, err, sizeof(err));
				cout << "Cannot open video encoder for stream. error def: " << err <<endl;
				return false;
			}

			avcodec_parameters_from_context(out_stream_space_texture->codecpar, enc_ctx_space_texture);

			cout << " stream index for space texture files: " << out_stream_space_texture->index << std::endl;

		}



		if (!(format_context->oformat->flags & AVFMT_NOFILE)) {
			ret = avio_open(&format_context->pb, filename, AVIO_FLAG_WRITE);
			if (ret < 0) {
				cout << "Could not open output file";
				return false;
			}
		}

		/* init muxer, write output file header */
		ret = avformat_write_header(format_context, NULL);
		if (ret < 0) {
			char err[4096];
			av_strerror(ret, err, sizeof(err));
			cout << "Error occurred when opening output file. Error: " << err << endl;
			return false;
		}


		//open container
		return true;
	}


	/**
	 * Encodes PLY data
	 *
	 * @param data
	 * contains 3D mesh data in PLY format
	 *
	 * @param length
	 * length of the data
	 *
	 * @param encoderBuffer
	 * Encoded data will be saved to encoderBuffer
	 *
	 * @param type
	 * type of the content OBJ or PLY
	 * const int OBJ_FORMAT = 0;
	 * const int PLY_FORMAT = 1
	 *
	 */
	bool encode(const char* data, size_t length, draco::EncoderBuffer *encoderBuffer, int type) {

		Mesh *m=nullptr;
		DecoderBuffer buffer;
		buffer.Init(data, length);
		bool decodedToMesh = false;

		if (type == PLY_FORMAT)
		{
//			Status status = plyDecoder.DecodeFromBuffer(&buffer, &m);
//			if (status.code() == Status::Code::OK) {
//				decodedToMesh = true;
//			}

                    decodedToMesh = plyDecoder.DecodeFromBuffer(&buffer, m);
		}
		else if (type == OBJ_FORMAT)
		{
			Status status = objDecoder.DecodeFromBuffer(&buffer, m);
			if (status.code() == Status::Code::OK) {
				decodedToMesh = true;
			}
		}

		if (decodedToMesh)
		{
			Status status = encoder.EncodeMeshToBuffer(*m, encoderBuffer);
			Status::Code code = status.code();
			if (code == Status::Code::OK) {
                                delete m;
				return true;
			}
		}
		else {
			cout << "Could not decode from buffer" << endl;
		}
                delete m;
		return false;
	}

        bool encodeFromFile(const std::string &file, draco::EncoderBuffer *encoderBuffer, int type) {


            draco::Mesh *mesh = nullptr;
            auto maybe_mesh = draco::ReadMeshFromFile(file, false );
            if (!maybe_mesh.ok()) {
                printf("Failed loading the input mesh: %s.\n",
                maybe_mesh.status().error_msg());
                return false;
            }
            mesh = maybe_mesh.value().get();




		    Status status = encoder.EncodeMeshToBuffer( *mesh, encoderBuffer);
			Status::Code code = status.code();
			if (code == Status::Code::OK) return true;
                        else return false;

        delete mesh;
        mesh=nullptr;
	}


	/**
	 * Encodes PLY Data and writes it to container that is opened before
	 *
	 * Stream index for ply data is 0
	 *
	 * @param data
	 * contains 3D mesh data in PLY format
	 *
	 * @param length
	 * length of the data
	 *
	 * @param timestamp
	 * timestamp of the data in microseconds format. Make first frame's timestamp  0(zero)
	 *
	 * @param type
	 * type of the content OBJ or PLY
	 * const int OBJ_FORMAT = 0;
	 * const int PLY_FORMAT = 1
	 *
	 * @return boolean
	 * true if it encodes data and writes to the file, otherwise it returns false
	 */


	/**
	 * Write data to container for PLY and OBJ files
	 */





	/**
	 * Encodes PLY/OBJ Data and writes it to container space container's mesh stream that is opened before
	 *
	 * Stream index for ply data is 2
	 *
	 * @param data
	 * contains 3D mesh data in PLY format
	 *
	 * @param length
	 * length of the data
	 *
	 * @param timestamp
	 * timestamp of the data in microseconds format. Make first frame's timestamp  0(zero)
	 *
	 * @param type
	 * type of the content OBJ or PLY
	 * const int OBJ_FORMAT = 0;
	 * const int PLY_FORMAT = 1
	 *
	 * @return boolean
	 * true if it encodes data and writes to the file, otherwise it returns false
	 */


	/**
	 * Writes texture file(PNG) to the container
	 *
	 * Stream index for texture files is 1
	 *
	 */



	bool writeTextureToContainer(char* data, int length, long timestamp, int streamIndex)
	{
		bool result = false;
		AVPacket enc_pkt;
		enc_pkt.data = NULL;
		enc_pkt.size = 0;

		av_init_packet(&enc_pkt);
		enc_pkt.stream_index = streamIndex;
		enc_pkt.data = (unsigned char*)data;
		enc_pkt.size = length;

		enc_pkt.pts =  av_rescale_q(timestamp, AV_TIME_BASE_Q, out_stream_texture->time_base);
		enc_pkt.dts = enc_pkt.pts;

		int ret = av_write_frame(format_context, &enc_pkt);
		if (ret >= 0) {
			result = true;
		}
		else {
			char errbuf[4096];
			av_strerror(ret, errbuf, sizeof(errbuf));
			cout << "Could not write to container. Error: " << errbuf << endl;
		}

		return result;
	}





	/**
	 *  Reads packets from file and stored in the data field
	 *
	 *  @param data
	 *  Decoded 3D Mesh will be stored with PLY format in to this pointer
	 *  if the read packet is compressed 3d mesh
	 *  or
	 *  PNG file will be stored in to this pointer if the read packet is
	 *  texture file
	 *
	 *  @param length
	 *  lenght of the decoded data
	 *
	 *  @param type
	 * type of the content OBJ or PLY
	 * const int OBJ_FORMAT = 0;
	 * const int PLY_FORMAT = 1
	 *
	 *  @return the index of the packet
	 *  if index is zero then packet is 3D Mesh
	 *  if index is 1 then packet is Texture file (PNG)
	 *
	 *  return -1 if it does not read packet successfully
	 */
	int read(char* &data, int &length, int type) {

		if (!isWriteMode)
		{

			AVPacket pkt;
			if (av_read_frame(format_context, &pkt)<0) {
				//cout << "Cannot read frame " << endl;
				return -1;
			}

//			if (pkt.stream_index == STREAM_INDEX_MESH || pkt.stream_index == STREAM_INDEX_SPACE_CONTAINER_MESH) {
//				draco::DecoderBuffer decoderBuffer;
//				decoderBuffer.Init((const char*)pkt.data, pkt.size);
//
//				StatusOr<std::unique_ptr<Mesh>> meshStatusOr = decoder.DecodeMeshFromBuffer(&decoderBuffer);
//
//				const Mesh* mesh = meshStatusOr.value().get();
//
//
//				if (!meshStatusOr.ok()) {
//					cout << "Failed to decode mesh from buffer. index: " << pkt.stream_index << endl;
//					return -1;
//				}
//				buffer.Clear();
//				if (type == PLY_FORMAT)
//				{
//					if (!plyEncoder.EncodeToBuffer(*mesh, &buffer)) {
//						cout << "Failed to encode mesh to buffer " << endl;
//						return -1;
//					}
//				}
//				else if (type == OBJ_FORMAT)
//				{
//					if (!objEncoder.EncodeToBuffer(*mesh, &buffer)) {
//						cout << "Failed to encode mesh to buffer " << endl;
//						return -1;
//					}
//				}
//				else {
//					return -1;
//				}
//
//				data = (char*)buffer.data();
//				length = buffer.size();
//			}
//			else if (pkt.stream_index == STREAM_INDEX_MESH_TEXTURE || pkt.stream_index == STREAM_INDEX_SPACE_CONTAINER_TEXTURE)
			if (pkt.stream_index == STREAM_INDEX_MESH_TEXTURE || pkt.stream_index == STREAM_INDEX_SPACE_CONTAINER_TEXTURE)

			{
				if (tmpDataSize < pkt.size) {
					if (tmpData == NULL) {
						delete[] tmpData;
					}
					tmpData = new char[pkt.size];
					tmpDataSize = pkt.size;
				}
				memcpy(tmpData, pkt.data, pkt.size);
				data = tmpData;
				length = pkt.size;
			}

			int streamIndex = pkt.stream_index;
			av_packet_unref(&pkt);
			return streamIndex;
		}
		return -1;

	}

	/**
	 * Closes the file. Call this function for both write and read mode
	 *
	 * @return boolean
	 * true if it closes file successfully, otherwise it returns false
	 */
	bool close() {
		if (format_context != NULL) {
			if (isWriteMode) {
				int ret;
				do  {
					//flush buffer
					ret = av_write_frame(format_context, NULL);
				}
				while(ret == 0);

				av_write_trailer(format_context);
				if (enc_ctx != NULL) {
					avcodec_close(enc_ctx);
				}
				avformat_free_context(format_context);
			}
			else {
				avformat_close_input(&format_context);
			}
		}
		if (tmpData != NULL) {
			delete[] tmpData;
		}
		return true;
	}


	int getTmpDataSize() {
		return tmpDataSize;
	}

private:

	AVFormatContext* format_context = NULL;
	AVCodecContext* enc_ctx = NULL;
	AVCodecContext* enc_ctx_texture = NULL;
	AVCodecContext* enc_ctx_space_mesh = NULL;
	AVCodecContext* enc_ctx_space_texture = NULL;
	draco::Encoder encoder;
	draco::Decoder decoder;
	draco::PlyDecoder plyDecoder;
	draco::PlyEncoder plyEncoder;
	draco::ObjEncoder objEncoder;
	draco::ObjDecoder objDecoder;
	draco::EncoderBuffer buffer;

	bool isWriteMode;
	//AVStream* out_stream;
	AVStream* out_stream_texture;
	//AVStream* out_stream_space_mesh;
	AVStream* out_stream_space_texture;
	char* tmpData = NULL;  //10MB
	int tmpDataSize = 0;
};


#endif /* THREE_D_DCONTAINER_H_ */
