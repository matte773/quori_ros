#ifndef _QUORI_FACE_TEXTURE_HPP_
#define _QUORI_FACE_TEXTURE_HPP_

#include <cstdint>
#include <memory>

namespace quori_face
{
  /**
   * \class Texture
   * 
   * An OpenGL texture object
   */
  class Texture
  {
  public:
    typedef std::shared_ptr<Texture> Ptr;
    typedef std::shared_ptr<const Texture> ConstPtr;

    ~Texture();

    /**
     * \fn create
     */
    static Ptr create(std::size_t rows, std::size_t cols, float *const data);
    
    /**
     * \fn create
     */
    static Ptr create(std::size_t rows, std::size_t cols, const std::uint32_t format, const uint8_t *const data);

    /**
     * \fn bind
     */
    void bind();

    /**
     * \fn getHandle
     * \return The underlying OpenGL handle
     */
    std::uint32_t getHandle() const;

  private:
    Texture(const std::uint32_t handle);
    std::uint32_t handle_;
  };
}

#endif